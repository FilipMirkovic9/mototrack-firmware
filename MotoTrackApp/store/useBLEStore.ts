import { create } from 'zustand';

interface Telemetry {
  lean: number;
  pitch: number;
  usbPresent: boolean;
  isCharging: boolean;
}

interface BLEState {
  isScanning: boolean;
  isConnected: boolean;
  deviceName: string | null;
  telemetry: Telemetry | null;
  nusLog: string[];
  setScanning: (v: boolean) => void;
  setConnected: (v: boolean) => void;
  setDeviceName: (name: string | null) => void;
  setTelemetry: (t: Telemetry) => void;
  appendNusLog: (line: string) => void;
  reset: () => void;
}

export const useBLEStore = create<BLEState>((set) => ({
  isScanning: false,
  isConnected: false,
  deviceName: null,
  telemetry: null,
  nusLog: [],
  setScanning: (v) => set({ isScanning: v }),
  setConnected: (v) => set({ isConnected: v }),
  setDeviceName: (name) => set({ deviceName: name }),
  setTelemetry: (t) => set({ telemetry: t }),
  appendNusLog: (line) =>
    set((state) => ({
      nusLog: [...state.nusLog.slice(-19), line],
    })),
  reset: () =>
    set({
      isScanning: false,
      isConnected: false,
      deviceName: null,
      telemetry: null,
      nusLog: [],
    }),
}));
