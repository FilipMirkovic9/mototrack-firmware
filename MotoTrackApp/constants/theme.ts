export const Colors = {
  background: '#FFFFFF',
  backgroundSecondary: '#F5F5F7',
  text: '#1C1C1E',
  textSecondary: '#8E8E93',
  accent: '#007AFF',
  border: '#E5E5EA',
  success: '#34C759',
  destructive: '#FF3B30',
  white: '#FFFFFF',
} as const;

export const Spacing = {
  xs: 4,
  sm: 8,
  md: 16,
  lg: 24,
  xl: 32,
} as const;

export const Typography = {
  largeTitle: { fontSize: 28, fontWeight: '700' as const, color: Colors.text },
  body: { fontSize: 16, color: Colors.text },
  caption: { fontSize: 13, color: Colors.textSecondary },
} as const;
