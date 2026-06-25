import { useRef, useCallback, useEffect } from 'react';
import { BleManager, Device, Subscription } from 'react-native-ble-plx';
import { useBLEStore } from '../store/useBLEStore';
import {
  DEVICE_NAME,
  SERVICE_UUID,
  TELEMETRY_CHAR_UUID,
  NUS_SERVICE_UUID,
  NUS_TX_UUID,
} from '../constants/ble';

const manager = new BleManager();

function base64ToDataView(base64: string): DataView {
  const binaryStr = atob(base64);
  const bytes = new Uint8Array(binaryStr.length);
  for (let i = 0; i < binaryStr.length; i++) {
    bytes[i] = binaryStr.charCodeAt(i);
  }
  return new DataView(bytes.buffer);
}

export function useBLE() {
  const deviceRef = useRef<Device | null>(null);
  const telemetrySubRef = useRef<Subscription | null>(null);
  const nusSubRef = useRef<Subscription | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const scanAndConnectRef = useRef<(() => Promise<void>) | null>(null);

  const { setScanning, setConnected, setDeviceName, setTelemetry, appendNusLog, reset } =
    useBLEStore();

  const clearReconnectTimer = useCallback(() => {
    if (reconnectTimerRef.current !== null) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }
  }, []);

  const disconnect = useCallback(async () => {
    clearReconnectTimer();
    manager.stopDeviceScan();
    telemetrySubRef.current?.remove();
    nusSubRef.current?.remove();
    telemetrySubRef.current = null;
    nusSubRef.current = null;
    if (deviceRef.current) {
      await deviceRef.current.cancelConnection().catch(() => undefined);
      deviceRef.current = null;
    }
    reset();
  }, [clearReconnectTimer, reset]);

  const scanAndConnect = useCallback(async () => {
    await disconnect();
    setScanning(true);

    manager.startDeviceScan(null, { allowDuplicates: false }, async (error, device) => {
      if (error) {
        setScanning(false);
        return;
      }
      if (!device || device.name !== DEVICE_NAME) return;

      manager.stopDeviceScan();
      setScanning(false);

      try {
        const connected = await device.connect();
        await connected.discoverAllServicesAndCharacteristics();
        deviceRef.current = connected;
        setConnected(true);
        setDeviceName(connected.name ?? DEVICE_NAME);

        manager.onDeviceDisconnected(connected.id, () => {
          setConnected(false);
          deviceRef.current = null;
          telemetrySubRef.current?.remove();
          nusSubRef.current?.remove();
          telemetrySubRef.current = null;
          nusSubRef.current = null;
          reconnectTimerRef.current = setTimeout(() => {
            scanAndConnectRef.current?.();
          }, 2000);
        });

        telemetrySubRef.current = connected.monitorCharacteristicForService(
          SERVICE_UUID,
          TELEMETRY_CHAR_UUID,
          (err, char) => {
            if (err || !char?.value) return;
            const dv = base64ToDataView(char.value);
            if (dv.byteLength < 12) return;
            setTelemetry({
              lean: dv.getFloat32(0, true),
              pitch: dv.getFloat32(4, true),
              usbPresent: (dv.getUint32(8, true) & 1) !== 0,
              isCharging: (dv.getUint32(8, true) & 2) !== 0,
            });
          },
        );

        nusSubRef.current = connected.monitorCharacteristicForService(
          NUS_SERVICE_UUID,
          NUS_TX_UUID,
          (err, char) => {
            if (err || !char?.value) return;
            const binaryStr = atob(char.value);
            appendNusLog(binaryStr.replace(/[\r\n]+$/, ''));
          },
        );
      } catch {
        setScanning(false);
        setConnected(false);
      }
    });
  }, [disconnect, setScanning, setConnected, setDeviceName, setTelemetry, appendNusLog]);

  useEffect(() => {
    scanAndConnectRef.current = scanAndConnect;
  }, [scanAndConnect]);

  const cleanup = useCallback(() => {
    clearReconnectTimer();
    disconnect();
  }, [clearReconnectTimer, disconnect]);

  return { scanAndConnect, disconnect, cleanup };
}
