import React from 'react';
import { AuthProvider } from '@site/src/components/auth/AuthProvider';

export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return <AuthProvider>{children}</AuthProvider>;
}
