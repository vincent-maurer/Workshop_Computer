import {
  ArrowDown,
  ArrowUp,
  Cable,
  CircleHelp,
  Download,
  Eraser,
  FileDown,
  FolderOpen,
  Pause,
  Play,
  Plus,
  Scissors,
  Moon,
  Sun,
  Terminal,
  Trash2,
  Upload,
  X,
} from 'lucide-react';
import { MouseEvent, useEffect, useMemo, useRef, useState } from 'react';
import { decodeAndEncodeFile, makePreviewBuffer } from './audio';
import {
  BANK_SAMPLE_RATE,
  BankSample,
  DEFAULT_CAPACITY_BYTES,
  buildBankBlob,
  croppedPcm,
  parseBankBlob,
  usedAudioBytes,
} from './bank';
import { DeviceInfo, StretchcoreSerial } from './serial';

const serial = new StretchcoreSerial();
const stretchcoreImageUrl = `${import.meta.env.BASE_URL}stretchcore.png`;
const stretchcoreUf2Url = `${import.meta.env.BASE_URL}stretchcore.uf2`;

type StatusKind = 'idle' | 'good' | 'warn' | 'bad';
type Theme = 'light' | 'dark';

interface Status {
  text: string;
  kind: StatusKind;
}

export function App() {
  const [samples, setSamples] = useState<BankSample[]>([]);
  const [device, setDevice] = useState<DeviceInfo | null>(null);
  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState<Status>({ text: 'Disconnected', kind: 'idle' });
  const [busy, setBusy] = useState(false);
  const [progress, setProgress] = useState(0);
  const [playingId, setPlayingId] = useState<string | null>(null);
  const [playheadFrame, setPlayheadFrame] = useState(0);
  const [debugLog, setDebugLog] = useState<string[]>([]);
  const [debugOpen, setDebugOpen] = useState(false);
  const [imageOpen, setImageOpen] = useState(false);
  const [theme, setTheme] = useState<Theme>(() => loadTheme());
  const audioRef = useRef<{
    context: AudioContext;
    source: AudioBufferSourceNode;
    startedAt: number;
    frameCount: number;
  } | null>(null);

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    window.localStorage.setItem('stretchcore-theme', theme);
  }, [theme]);

  useEffect(() => {
    if (!imageOpen) return;
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') setImageOpen(false);
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [imageOpen]);

  useEffect(() => {
    if (!playingId || !audioRef.current) {
      setPlayheadFrame(0);
      return;
    }

    let raf = 0;
    const tick = () => {
      const active = audioRef.current;
      if (!active) return;
      const elapsed = active.context.currentTime - active.startedAt;
      const frame = Math.floor((elapsed * BANK_SAMPLE_RATE) % Math.max(1, active.frameCount));
      setPlayheadFrame(frame);
      raf = window.requestAnimationFrame(tick);
    };
    raf = window.requestAnimationFrame(tick);
    return () => window.cancelAnimationFrame(raf);
  }, [playingId]);

  const capacity = device?.capacityBytes || DEFAULT_CAPACITY_BYTES;
  const used = useMemo(() => usedAudioBytes(samples), [samples]);
  const overCapacity = used > capacity;
  const usageRatio = capacity > 0 ? Math.min(1, used / capacity) : 0;
  const showStatusSpinner = busy && status.kind === 'idle';

  async function connect() {
    if (connected) {
      stopPreview();
      await serial.disconnect();
      setConnected(false);
      setDevice(null);
      setStatus({ text: 'Disconnected', kind: 'idle' });
      return;
    }

    try {
      setBusy(true);
      setDebugLog([]);
      serial.setLogger((message) => {
        setDebugLog((current) => [...current.slice(-80), `${new Date().toLocaleTimeString()} ${message}`]);
      });
      setStatus({ text: 'Connecting', kind: 'idle' });
      await serial.connect();
      const info = await initialiseWithRetry();
      setDevice(info);
      setConnected(true);
      setStatus({ text: 'Downloading', kind: 'idle' });
      const bank = await serial.readBank((ratio) => setProgress(ratio));
      if (bank) {
        const parsed = parseBankBlob(bank);
        setSamples(parsed.samples);
        setStatus({ text: `Loaded ${parsed.samples.length} samples from device`, kind: 'good' });
      } else {
        setSamples([]);
        setStatus({ text: 'Connected: device bank is empty', kind: 'good' });
      }
      setProgress(0);
    } catch (error) {
      setStatus({ text: errorMessage(error), kind: 'bad' });
      try {
        await serial.disconnect();
      } catch (_) {}
      serial.setLogger(null);
      setConnected(false);
    } finally {
      setBusy(false);
    }
  }

  async function initialiseWithRetry(): Promise<DeviceInfo> {
    let lastError: unknown = null;
    for (let attempt = 0; attempt < 4; attempt++) {
      try {
        setStatus({ text: attempt === 0 ? 'Connecting' : 'Retrying connection', kind: 'idle' });
        await serial.sync();
        await new Promise((resolve) => setTimeout(resolve, 120));
        return await serial.info(true);
      } catch (error) {
        lastError = error;
        await new Promise((resolve) => setTimeout(resolve, 250 * (attempt + 1)));
      }
    }
    throw lastError instanceof Error ? lastError : new Error(String(lastError));
  }

  async function addFiles(fileList: FileList | File[]) {
    const files = Array.from(fileList).filter((file) => file.type.startsWith('audio/') || /\.(aif|aiff|wav|mp3|flac|ogg)$/i.test(file.name));
    if (files.length === 0) return;
    setBusy(true);
    try {
      const next: BankSample[] = [];
      for (const file of files) {
        setStatus({ text: `Processing ${file.name}`, kind: 'idle' });
        next.push(await decodeAndEncodeFile(file));
      }
      setSamples((current) => [...current, ...next].slice(0, 32));
      setStatus({ text: `Added ${next.length} sample${next.length === 1 ? '' : 's'}`, kind: 'good' });
    } catch (error) {
      setStatus({ text: errorMessage(error), kind: 'bad' });
    } finally {
      setBusy(false);
    }
  }

  async function uploadBank() {
    if (!connected) return;
    try {
      setBusy(true);
      stopPreview();
      setStatus({ text: 'Uploading', kind: 'idle' });
      const blob = buildBankBlob(samples, capacity);
      await serial.writeBank(blob, (ratio) => setProgress(ratio));
      const info = await serial.info();
      setDevice(info);
      setProgress(0);
      setStatus({ text: 'Upload complete', kind: 'good' });
    } catch (error) {
      setStatus({ text: errorMessage(error), kind: 'bad' });
    } finally {
      setBusy(false);
    }
  }

  async function readDevice() {
    if (!connected) return;
    try {
      setBusy(true);
      stopPreview();
      setStatus({ text: 'Downloading', kind: 'idle' });
      const bank = await serial.readBank((ratio) => setProgress(ratio));
      const info = await serial.info();
      setDevice(info);
      if (bank) {
        const parsed = parseBankBlob(bank);
        setSamples(parsed.samples);
        setStatus({ text: `Read ${parsed.samples.length} samples`, kind: 'good' });
      } else {
        setSamples([]);
        setStatus({ text: 'Device bank is empty', kind: 'good' });
      }
      setProgress(0);
    } catch (error) {
      setStatus({ text: errorMessage(error), kind: 'bad' });
    } finally {
      setBusy(false);
    }
  }

  async function eraseDevice() {
    if (!connected) return;
    try {
      setBusy(true);
      stopPreview();
      setStatus({ text: 'Erasing', kind: 'idle' });
      await serial.erase();
      const info = await serial.info();
      setDevice(info);
      setSamples([]);
      setStatus({ text: 'Device bank erased', kind: 'good' });
    } catch (error) {
      setStatus({ text: errorMessage(error), kind: 'bad' });
    } finally {
      setBusy(false);
    }
  }

  function updateSample(id: string, patch: Partial<BankSample>) {
    setSamples((current) => current.map((sample) => (sample.id === id ? { ...sample, ...patch } : sample)));
  }

  function removeSample(id: string) {
    if (playingId === id) stopPreview();
    setSamples((current) => current.filter((sample) => sample.id !== id));
  }

  function moveSample(index: number, direction: -1 | 1) {
    setSamples((current) => {
      const next = [...current];
      const target = index + direction;
      if (target < 0 || target >= next.length) return current;
      [next[index], next[target]] = [next[target], next[index]];
      return next;
    });
  }

  function cropSample(id: string) {
    setSamples((current) =>
      current.map((sample) => {
        if (sample.id !== id) return sample;
        const pcm = croppedPcm(sample);
        return { ...sample, pcm, cropStart: 0, cropEnd: pcm.length };
      }),
    );
  }

  function stopPreview() {
    audioRef.current?.source.stop();
    audioRef.current?.context.close();
    audioRef.current = null;
    setPlayingId(null);
  }

  async function togglePreview(sample: BankSample) {
    if (playingId === sample.id) {
      stopPreview();
      return;
    }
    stopPreview();
    const context = new AudioContext();
    const pcm = croppedPcm(sample);
    const source = context.createBufferSource();
    source.buffer = makePreviewBuffer(context, pcm);
    source.loop = true;
    source.connect(context.destination);
    const startedAt = context.currentTime;
    source.start();
    source.onended = () => {
      if (audioRef.current?.source === source) setPlayingId(null);
    };
    audioRef.current = { context, source, startedAt, frameCount: pcm.length };
    setPlayingId(sample.id);
  }

  return (
    <main className="app">
      <header className="topbar">
        <div>
          <h1>stretchcore loader</h1>
          <div className={`status ${status.kind}`}>
            {showStatusSpinner ? <span className="spinner" aria-hidden="true" /> : null}
            {status.text}
          </div>
        </div>
        <div className="toolbar">
          <button className="primary" onClick={connect} disabled={busy} title={connected ? 'Disconnect from device' : 'Connect to device'}>
            <Cable size={18} />
            {connected ? 'Disconnect' : 'Connect'}
          </button>
          <label className={`button ${busy ? 'disabled' : ''}`} title="Add audio files">
            <Plus size={18} />
            Add
            <input
              type="file"
              multiple
              accept="audio/*,.aif,.aiff"
              disabled={busy}
              onChange={(event) => {
                if (event.target.files) void addFiles(event.target.files);
                event.currentTarget.value = '';
              }}
            />
          </label>
          <button onClick={uploadBank} disabled={!connected || busy || overCapacity || samples.length === 0} title="Upload samples to device">
            <Upload size={18} />
            Upload
          </button>
          <button onClick={readDevice} disabled={!connected || busy} title="Download samples from device">
            <Download size={18} />
            Read
          </button>
          <button onClick={eraseDevice} disabled={!connected || busy} title="Erase samples on device">
            <Eraser size={18} />
            Erase
          </button>
          <a className="button" href={stretchcoreUf2Url} download="stretchcore.uf2" title="Download firmware UF2">
            <FileDown size={18} />
            Firmware
          </a>
          <div className="toolbar-spacer" />
          <button
            className="icon-button"
            onClick={() => setImageOpen(true)}
            title="Show module image"
            aria-label="Show module image"
          >
            <CircleHelp size={18} />
          </button>
          <button
            className="icon-button"
            onClick={() => setDebugOpen((current) => !current)}
            title={debugOpen ? 'Hide serial debug log' : 'Show serial debug log'}
            aria-label={debugOpen ? 'Hide serial debug log' : 'Show serial debug log'}
            aria-pressed={debugOpen}
          >
            <Terminal size={18} />
          </button>
          <button
            className="icon-button"
            onClick={() => setTheme((current) => (current === 'dark' ? 'light' : 'dark'))}
            title={theme === 'dark' ? 'Switch to light mode' : 'Switch to dark mode'}
            aria-label={theme === 'dark' ? 'Switch to light mode' : 'Switch to dark mode'}
          >
            {theme === 'dark' ? <Sun size={18} /> : <Moon size={18} />}
          </button>
        </div>
      </header>

      {imageOpen ? (
        <div className="modal-backdrop" role="presentation" onMouseDown={() => setImageOpen(false)}>
          <div
            className="image-modal"
            role="dialog"
            aria-modal="true"
            aria-label="stretchcore image"
            onMouseDown={(event) => event.stopPropagation()}
          >
            <button className="icon-button modal-close" onClick={() => setImageOpen(false)} title="Close" aria-label="Close">
              <X size={18} />
            </button>
            <img src={stretchcoreImageUrl} alt="stretchcore" />
          </div>
        </div>
      ) : null}

      <section className="capacity">
        <div className="capacity-line">
          <span>{formatBytes(used)} used</span>
          <span>{formatBytes(Math.max(0, capacity - used))} free</span>
          <span>
            {samples.length} sample{samples.length === 1 ? '' : 's'}
          </span>
          <span>{device ? `FW ${device.firmware}` : 'No device'}</span>
        </div>
        <div className="meter">
          <div className={overCapacity ? 'over' : ''} style={{ width: `${usageRatio * 100}%` }} />
        </div>
        {busy && progress > 0 ? <div className="transfer" style={{ width: `${progress * 100}%` }} /> : null}
      </section>

      {debugOpen ? (
        <section className="debug-log">
          <div className="debug-title">Serial debug</div>
          {debugLog.length > 0 ? (
            debugLog.map((entry, index) => <div key={`${entry}-${index}`}>{entry}</div>)
          ) : (
            <div className="debug-empty">No serial messages yet</div>
          )}
        </section>
      ) : null}

      <section
        className="sample-list"
        onDragOver={(event) => event.preventDefault()}
        onDrop={(event) => {
          event.preventDefault();
          void addFiles(event.dataTransfer.files);
        }}
      >
        {samples.length === 0 ? (
          <div className="empty">
            <FolderOpen size={28} />
            <span>No samples loaded</span>
          </div>
        ) : (
          samples.map((sample, index) => (
            <SampleRow
              key={sample.id}
              sample={sample}
              index={index}
              playing={playingId === sample.id}
              playheadFrame={playingId === sample.id ? playheadFrame : null}
              theme={theme}
              onUpdate={(patch) => updateSample(sample.id, patch)}
              onRemove={() => removeSample(sample.id)}
              onMove={(direction) => moveSample(index, direction)}
              onCrop={() => cropSample(sample.id)}
              onPreview={() => void togglePreview(sample)}
            />
          ))
        )}
      </section>
    </main>
  );
}

function SampleRow({
  sample,
  index,
  playing,
  playheadFrame,
  theme,
  onUpdate,
  onRemove,
  onMove,
  onCrop,
  onPreview,
}: {
  sample: BankSample;
  index: number;
  playing: boolean;
  playheadFrame: number | null;
  theme: Theme;
  onUpdate: (patch: Partial<BankSample>) => void;
  onRemove: () => void;
  onMove: (direction: -1 | 1) => void;
  onCrop: () => void;
  onPreview: () => void;
}) {
  const length = croppedPcm(sample).length;
  return (
    <article className="sample-row">
      <div className="sample-meta">
        <span className="slot">{index + 1}</span>
        <input
          className="name"
          value={sample.name}
          maxLength={47}
          onChange={(event) => onUpdate({ name: event.target.value })}
        />
        <label>
          BPM
          <input
            className="bpm"
            type="number"
            min={1}
            max={65535}
            value={sample.bpm}
            onChange={(event) => onUpdate({ bpm: Number(event.target.value) || 1 })}
          />
        </label>
        <span>{formatDuration(length)}</span>
        <span>{formatBytes(length)}</span>
      </div>
      <Waveform sample={sample} playheadFrame={playheadFrame} theme={theme} onUpdate={onUpdate} />
      <div className="row-actions">
        <button onClick={onPreview} title={playing ? 'Stop sample preview' : 'Play sample preview'}>
          {playing ? <Pause size={16} /> : <Play size={16} />}
        </button>
        <button onClick={onCrop} disabled={sample.cropStart === 0 && sample.cropEnd === sample.pcm.length} title="Crop sample to selection">
          <Scissors size={16} />
        </button>
        <button onClick={() => onMove(-1)} disabled={index === 0} title="Move sample up">
          <ArrowUp size={16} />
        </button>
        <button onClick={() => onMove(1)} title="Move sample down">
          <ArrowDown size={16} />
        </button>
        <button onClick={onRemove} title="Remove sample">
          <Trash2 size={16} />
        </button>
      </div>
    </article>
  );
}

function Waveform({
  sample,
  playheadFrame,
  theme,
  onUpdate,
}: {
  sample: BankSample;
  playheadFrame: number | null;
  theme: Theme;
  onUpdate: (patch: Partial<BankSample>) => void;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const dragStart = useRef<number | null>(null);

  function draw(canvas: HTMLCanvasElement) {
    const rect = canvas.getBoundingClientRect();
    const scale = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.floor(rect.width * scale));
    const height = Math.max(1, Math.floor(rect.height * scale));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const waveBackground = theme === 'dark' ? '#191918' : '#f7f7f4';
    const waveLine = theme === 'dark' ? '#f1f0ea' : '#202020';
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = waveBackground;
    ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = waveLine;
    ctx.lineWidth = Math.max(1, scale);
    ctx.beginPath();
    const mid = height / 2;
    for (let x = 0; x < width; x++) {
      const start = Math.floor((x / width) * sample.pcm.length);
      const end = Math.max(start + 1, Math.floor(((x + 1) / width) * sample.pcm.length));
      let min = 0;
      let max = 0;
      for (let i = start; i < end; i++) {
        const value = sample.pcm[i] ? ((sample.pcm[i] << 24) >> 24) / 128 : 0;
        min = Math.min(min, value);
        max = Math.max(max, value);
      }
      ctx.moveTo(x, mid + min * mid * 0.9);
      ctx.lineTo(x, mid + max * mid * 0.9);
    }
    ctx.stroke();

    const left = (sample.cropStart / sample.pcm.length) * width;
    const right = (sample.cropEnd / sample.pcm.length) * width;
    const isPartialSelection = sample.cropStart > 0 || sample.cropEnd < sample.pcm.length;
    if (isPartialSelection) {
      ctx.fillStyle = colorWithAlpha(waveLine, 0.14);
      ctx.fillRect(0, 0, left, height);
      ctx.fillRect(right, 0, Math.max(0, width - right), height);
      ctx.fillStyle = waveLine;
      ctx.fillRect(left, 0, Math.max(2, 2 * scale), height);
      ctx.fillRect(right, 0, Math.max(2, 2 * scale), height);
    }

    if (playheadFrame != null) {
      const playhead = ((sample.cropStart + playheadFrame) / sample.pcm.length) * width;
      ctx.fillStyle = cssVar('--playhead', '#d23b2a');
      ctx.fillRect(playhead, 0, Math.max(2, 2 * scale), height);
    }
  }

  useEffect(() => {
    if (canvasRef.current) draw(canvasRef.current);
  }, [sample, playheadFrame, theme]);

  function frameFromEvent(event: MouseEvent<HTMLCanvasElement>): number {
    const rect = event.currentTarget.getBoundingClientRect();
    const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
    return Math.round(ratio * sample.pcm.length);
  }

  function setSelection(start: number, end: number) {
    const cropStart = Math.max(0, Math.min(sample.pcm.length - 1, Math.min(start, end)));
    const cropEnd = Math.max(cropStart + 1, Math.min(sample.pcm.length, Math.max(start, end)));
    onUpdate({ cropStart, cropEnd });
  }

  return (
    <canvas
      ref={(node) => {
        canvasRef.current = node;
        if (node) draw(node);
      }}
      className="waveform"
      onMouseDown={(event) => {
        dragStart.current = frameFromEvent(event);
        setSelection(dragStart.current, dragStart.current + 1);
      }}
      onDoubleClick={() => {
        dragStart.current = null;
        onUpdate({ cropStart: 0, cropEnd: sample.pcm.length });
      }}
      onMouseMove={(event) => {
        if (dragStart.current == null) return;
        setSelection(dragStart.current, frameFromEvent(event));
      }}
      onMouseUp={() => {
        dragStart.current = null;
      }}
      onMouseLeave={() => {
        dragStart.current = null;
      }}
    />
  );
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
}

function formatDuration(frames: number): string {
  return `${(frames / BANK_SAMPLE_RATE).toFixed(2)} s`;
}

function loadTheme(): Theme {
  const stored =
    window.localStorage.getItem('stretchcore-theme') ?? window.localStorage.getItem('breaky-theme');
  if (stored === 'light' || stored === 'dark') return stored;
  return window.matchMedia?.('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
}

function cssVar(name: string, fallback: string): string {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

function colorWithAlpha(color: string, alpha: number): string {
  if (!color.startsWith('#')) return color;
  const hex = color.slice(1);
  const full =
    hex.length === 3
      ? hex
          .split('')
          .map((part) => part + part)
          .join('')
      : hex;
  const value = Number.parseInt(full, 16);
  if (!Number.isFinite(value)) return color;
  const r = (value >> 16) & 0xff;
  const g = (value >> 8) & 0xff;
  const b = value & 0xff;
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}
