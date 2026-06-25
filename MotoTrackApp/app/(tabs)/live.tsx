import React from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  ScrollView,
  StyleSheet,
  SafeAreaView,
} from 'react-native';
import { useBLEStore } from '../../store/useBLEStore';
import { useBLE } from '../../hooks/useBLE';
import { TelemetryCard } from '../../components/TelemetryCard';
import { ConnectionBadge } from '../../components/ConnectionBadge';
import { Colors, Spacing } from '../../constants/theme';

function getLeanDirection(lean: number): string {
  if (Math.abs(lean) < 3) return 'UPRIGHT';
  return lean < 0 ? 'LEFT' : 'RIGHT';
}

function getConnectionStatus(
  isScanning: boolean,
  isConnected: boolean,
): 'connected' | 'scanning' | 'disconnected' {
  if (isConnected) return 'connected';
  if (isScanning) return 'scanning';
  return 'disconnected';
}

export default function LiveScreen() {
  const { isScanning, isConnected, telemetry, nusLog } = useBLEStore();
  const { scanAndConnect } = useBLE();

  const status = getConnectionStatus(isScanning, isConnected);

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>Live</Text>
        <ConnectionBadge status={status} />
      </View>

      {!isConnected ? (
        <View style={styles.emptyState}>
          <Text style={styles.emptyIcon}>📡</Text>
          <Text style={styles.emptyTitle}>No Device</Text>
          <Text style={styles.emptySubtitle}>
            Tap below to scan for your MotoTrack device
          </Text>
          <TouchableOpacity
            style={[styles.scanButton, isScanning && styles.scanButtonScanning]}
            onPress={scanAndConnect}
            disabled={isScanning}
          >
            <Text style={styles.scanButtonText}>
              {isScanning ? 'Scanning...' : 'Scan for Device'}
            </Text>
          </TouchableOpacity>
        </View>
      ) : (
        <ScrollView contentContainerStyle={styles.telemetryContent}>
          {telemetry !== null ? (
            <>
              <View style={styles.leanContainer}>
                <Text style={styles.leanValue}>{telemetry.lean.toFixed(1)}°</Text>
                <Text style={styles.leanDirection}>
                  {getLeanDirection(telemetry.lean)}
                </Text>
              </View>

              <View style={styles.cardRow}>
                <TelemetryCard
                  label="Pitch"
                  value={telemetry.pitch.toFixed(1)}
                  unit="°"
                  fullWidth
                />
              </View>

              <View style={styles.indicatorRow}>
                <View
                  style={[
                    styles.indicator,
                    telemetry.usbPresent && styles.indicatorActive,
                  ]}
                >
                  <Text style={styles.indicatorText}>
                    USB {telemetry.usbPresent ? 'Connected' : 'Disconnected'}
                  </Text>
                </View>
                <View
                  style={[
                    styles.indicator,
                    telemetry.isCharging && styles.indicatorActive,
                  ]}
                >
                  <Text style={styles.indicatorText}>
                    {telemetry.isCharging ? 'Charging' : 'Not Charging'}
                  </Text>
                </View>
              </View>
            </>
          ) : (
            <View style={styles.waitingContainer}>
              <Text style={styles.waitingText}>Waiting for telemetry...</Text>
            </View>
          )}

          <View style={styles.nusSection}>
            <Text style={styles.nusSectionTitle}>Device Log</Text>
            <View style={styles.nusLog}>
              {nusLog.length === 0 ? (
                <Text style={styles.nusEmpty}>No messages</Text>
              ) : (
                nusLog.map((line, i) => (
                  <Text key={i} style={styles.nusLine}>
                    {line}
                  </Text>
                ))
              )}
            </View>
          </View>
        </ScrollView>
      )}
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.backgroundSecondary,
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: Spacing.md,
    paddingTop: Spacing.md,
    paddingBottom: Spacing.sm,
    backgroundColor: Colors.background,
    borderBottomWidth: StyleSheet.hairlineWidth,
    borderBottomColor: Colors.border,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
    color: Colors.text,
  },
  emptyState: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: Spacing.xl,
  },
  emptyIcon: {
    fontSize: 52,
    marginBottom: Spacing.md,
  },
  emptyTitle: {
    fontSize: 22,
    fontWeight: '700',
    color: Colors.text,
    marginBottom: 8,
  },
  emptySubtitle: {
    fontSize: 15,
    color: Colors.textSecondary,
    textAlign: 'center',
    marginBottom: Spacing.xl,
  },
  scanButton: {
    backgroundColor: Colors.accent,
    paddingHorizontal: 36,
    paddingVertical: 14,
    borderRadius: 14,
  },
  scanButtonScanning: {
    backgroundColor: Colors.textSecondary,
  },
  scanButtonText: {
    color: Colors.white,
    fontSize: 16,
    fontWeight: '600',
  },
  telemetryContent: {
    padding: Spacing.md,
    rowGap: Spacing.md,
  },
  leanContainer: {
    alignItems: 'center',
    paddingVertical: Spacing.lg,
  },
  leanValue: {
    fontSize: 80,
    fontWeight: '700',
    color: Colors.text,
    lineHeight: 90,
  },
  leanDirection: {
    fontSize: 18,
    fontWeight: '600',
    color: Colors.textSecondary,
    letterSpacing: 3,
    marginTop: 4,
  },
  cardRow: {
    flexDirection: 'row',
  },
  indicatorRow: {
    flexDirection: 'row',
    columnGap: Spacing.md,
  },
  indicator: {
    flex: 1,
    backgroundColor: Colors.background,
    borderRadius: 10,
    padding: Spacing.md,
    alignItems: 'center',
    borderWidth: 1,
    borderColor: Colors.border,
  },
  indicatorActive: {
    borderColor: Colors.accent,
    backgroundColor: Colors.accent + '12',
  },
  indicatorText: {
    fontSize: 14,
    fontWeight: '500',
    color: Colors.text,
  },
  waitingContainer: {
    alignItems: 'center',
    paddingVertical: Spacing.xl,
  },
  waitingText: {
    fontSize: 16,
    color: Colors.textSecondary,
  },
  nusSection: {
    marginTop: Spacing.xs,
  },
  nusSectionTitle: {
    fontSize: 13,
    fontWeight: '600',
    color: Colors.textSecondary,
    textTransform: 'uppercase',
    letterSpacing: 0.5,
    marginBottom: 6,
  },
  nusLog: {
    backgroundColor: '#1C1C1E',
    borderRadius: 10,
    padding: Spacing.md,
    minHeight: 100,
  },
  nusEmpty: {
    color: '#636366',
    fontFamily: 'Courier New',
    fontSize: 13,
  },
  nusLine: {
    color: '#E5E5EA',
    fontFamily: 'Courier New',
    fontSize: 13,
    lineHeight: 20,
  },
});
