import React from 'react';
import { View, Text, FlatList, StyleSheet, SafeAreaView } from 'react-native';
import { Colors, Spacing } from '../../constants/theme';

interface Ride {
  id: string;
  name: string;
  date: string;
  duration: string;
  maxLean: number;
  distance: number;
}

const MOCK_RIDES: Ride[] = [
  {
    id: '1',
    name: 'Morning Twisties',
    date: 'Jun 14 2026',
    duration: '1h 23m',
    maxLean: 47,
    distance: 89,
  },
  {
    id: '2',
    name: 'Evening Run',
    date: 'Jun 18 2026',
    duration: '0h 44m',
    maxLean: 38,
    distance: 52,
  },
  {
    id: '3',
    name: 'Sunday Blast',
    date: 'Jun 22 2026',
    duration: '2h 07m',
    maxLean: 51,
    distance: 134,
  },
];

function RideRow({ item }: { item: Ride }) {
  return (
    <View style={styles.row}>
      <View style={styles.rowLeft}>
        <Text style={styles.rideName}>{item.name}</Text>
        <Text style={styles.rideMeta}>
          {item.date} · {item.duration} · {item.distance} km
        </Text>
      </View>
      <View style={styles.leanBadge}>
        <Text style={styles.leanText}>{item.maxLean}°</Text>
      </View>
    </View>
  );
}

function EmptyState() {
  return (
    <View style={styles.emptyState}>
      <Text style={styles.emptyTitle}>No rides recorded yet</Text>
      <Text style={styles.emptySubtitle}>
        Connect your MotoTrack device and start a ride
      </Text>
    </View>
  );
}

export default function RidesScreen() {
  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>Rides</Text>
      </View>
      <FlatList
        data={MOCK_RIDES}
        keyExtractor={(item) => item.id}
        renderItem={({ item }) => <RideRow item={item} />}
        ListEmptyComponent={<EmptyState />}
        ItemSeparatorComponent={() => <View style={styles.separator} />}
        contentContainerStyle={MOCK_RIDES.length === 0 ? styles.emptyContainer : undefined}
      />
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
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: Colors.background,
    paddingHorizontal: Spacing.md,
    paddingVertical: Spacing.md,
  },
  rowLeft: {
    flex: 1,
    marginRight: Spacing.md,
  },
  rideName: {
    fontSize: 16,
    fontWeight: '600',
    color: Colors.text,
  },
  rideMeta: {
    fontSize: 13,
    color: Colors.textSecondary,
    marginTop: 2,
  },
  leanBadge: {
    backgroundColor: Colors.accent + '18',
    borderRadius: 8,
    paddingHorizontal: 10,
    paddingVertical: 4,
  },
  leanText: {
    fontSize: 15,
    fontWeight: '700',
    color: Colors.accent,
  },
  separator: {
    height: StyleSheet.hairlineWidth,
    backgroundColor: Colors.border,
    marginLeft: Spacing.md,
  },
  emptyContainer: {
    flex: 1,
  },
  emptyState: {
    alignItems: 'center',
    paddingTop: 80,
    paddingHorizontal: Spacing.xl,
  },
  emptyTitle: {
    fontSize: 18,
    fontWeight: '600',
    color: Colors.text,
  },
  emptySubtitle: {
    fontSize: 14,
    color: Colors.textSecondary,
    textAlign: 'center',
    marginTop: 8,
  },
});
