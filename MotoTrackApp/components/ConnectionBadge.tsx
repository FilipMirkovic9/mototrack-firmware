import React from 'react';
import { View, Text, StyleSheet } from 'react-native';

interface Props {
  status: 'connected' | 'scanning' | 'disconnected';
}

const STATUS_CONFIG = {
  connected: { label: 'Connected', color: '#34C759' },
  scanning: { label: 'Scanning...', color: '#007AFF' },
  disconnected: { label: 'Not Connected', color: '#8E8E93' },
} as const;

export function ConnectionBadge({ status }: Props) {
  const { label, color } = STATUS_CONFIG[status];
  return (
    <View style={[styles.pill, { backgroundColor: color + '22' }]}>
      <View style={[styles.dot, { backgroundColor: color }]} />
      <Text style={[styles.label, { color }]}>{label}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  pill: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 10,
    paddingVertical: 5,
    borderRadius: 20,
    gap: 5,
  },
  dot: {
    width: 6,
    height: 6,
    borderRadius: 3,
  },
  label: {
    fontSize: 13,
    fontWeight: '500',
  },
});
