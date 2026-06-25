import React from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  SafeAreaView,
  Alert,
} from 'react-native';
import Constants from 'expo-constants';
import { useBLEStore } from '../../store/useBLEStore';
import { Colors, Spacing } from '../../constants/theme';

function SectionHeader({ title }: { title: string }) {
  return <Text style={styles.sectionHeader}>{title}</Text>;
}

function SettingsRow({
  label,
  value,
  onPress,
  destructive,
}: {
  label: string;
  value?: string;
  onPress?: () => void;
  destructive?: boolean;
}) {
  return (
    <TouchableOpacity
      style={styles.row}
      onPress={onPress}
      disabled={onPress === undefined}
      activeOpacity={onPress !== undefined ? 0.6 : 1}
    >
      <Text style={[styles.rowLabel, destructive === true && styles.destructiveLabel]}>
        {label}
      </Text>
      {value !== undefined && <Text style={styles.rowValue}>{value}</Text>}
    </TouchableOpacity>
  );
}

function Divider() {
  return <View style={styles.divider} />;
}

export default function SettingsScreen() {
  const { deviceName } = useBLEStore();
  const appVersion = Constants.expoConfig?.version ?? '1.0.0';

  function handleClearHistory() {
    Alert.alert(
      'Clear Ride History',
      'This will permanently delete all saved rides. This action cannot be undone.',
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Delete All', style: 'destructive', onPress: () => {} },
      ],
    );
  }

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>Settings</Text>
      </View>

      <View style={styles.body}>
        <SectionHeader title="DEVICE" />
        <View style={styles.section}>
          <SettingsRow label="Device Name" value={deviceName ?? '—'} />
          <Divider />
          <SettingsRow label="Firmware Version" value="—" />
          <Divider />
          <SettingsRow label="Last Connected" value="—" />
        </View>

        <SectionHeader title="APP" />
        <View style={styles.section}>
          <SettingsRow label="App Version" value={appVersion} />
          <Divider />
          <SettingsRow
            label="Clear Ride History"
            onPress={handleClearHistory}
            destructive
          />
        </View>

        <SectionHeader title="ABOUT" />
        <View style={styles.section}>
          <View style={styles.aboutRow}>
            <Text style={styles.aboutTitle}>MotoTrack</Text>
            <Text style={styles.aboutSubtitle}>
              Motorcycle telemetry and navigation
            </Text>
          </View>
        </View>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: Colors.backgroundSecondary,
  },
  header: {
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
  body: {
    padding: Spacing.md,
  },
  sectionHeader: {
    fontSize: 13,
    fontWeight: '500',
    color: Colors.textSecondary,
    letterSpacing: 0.5,
    marginTop: Spacing.lg,
    marginBottom: 6,
    marginLeft: 4,
  },
  section: {
    backgroundColor: Colors.background,
    borderRadius: 12,
    overflow: 'hidden',
  },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: Spacing.md,
    paddingVertical: 13,
  },
  rowLabel: {
    fontSize: 16,
    color: Colors.text,
  },
  rowValue: {
    fontSize: 16,
    color: Colors.textSecondary,
  },
  destructiveLabel: {
    color: Colors.destructive,
  },
  divider: {
    height: StyleSheet.hairlineWidth,
    backgroundColor: Colors.border,
    marginLeft: Spacing.md,
  },
  aboutRow: {
    paddingHorizontal: Spacing.md,
    paddingVertical: Spacing.md,
  },
  aboutTitle: {
    fontSize: 16,
    fontWeight: '600',
    color: Colors.text,
  },
  aboutSubtitle: {
    fontSize: 14,
    color: Colors.textSecondary,
    marginTop: 2,
  },
});
