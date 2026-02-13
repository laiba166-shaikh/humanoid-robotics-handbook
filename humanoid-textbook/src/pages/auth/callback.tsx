import { FRONTEND_URL } from '@site/src/config';
import { useEffect, useState } from 'react';

export default function CallbackPage(): JSX.Element {
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Extract tokens from URL fragment
    const hash = window.location.hash.substring(1);
    const params = new URLSearchParams(hash);

    const accessToken = params.get('access_token');
    const refreshToken = params.get('refresh_token');

    if (!accessToken || !refreshToken) {
      setError('OAuth callback failed: missing tokens');
      setTimeout(() => {
        window.location.href = `${FRONTEND_URL}/auth/login?error=oauth_failed`;
      }, 2000);
      return;
    }

    // Store refresh token — AuthProvider will pick it up on the next page load
    localStorage.setItem('refresh_token', refreshToken);

    // Redirect to docs (full reload triggers AuthProvider.restoreSession)
    window.location.href = `${FRONTEND_URL}/docs`;
  }, []);

  if (error) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Authentication Error</h1>
        <p>{error}</p>
        <p>Redirecting to login...</p>
      </div>
    );
  }

  return (
    <div style={{ padding: '2rem', textAlign: 'center' }}>
      <h1>Completing Sign In</h1>
      <p>Please wait...</p>
    </div>
  );
}
