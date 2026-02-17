import React, { useState, useCallback, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useColorMode } from '@docusaurus/theme-common';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { useAuth } from '@site/src/components/auth/AuthProvider';
import { API_URL } from '@site/src/config';
import styles from './ChatWidget.module.css';

const CHATKIT_CDN = 'https://cdn.platform.openai.com/deployments/chatkit/chatkit.js';

/** Load the ChatKit web component definition from the OpenAI CDN (once). */
function useLoadChatKitScript(): boolean {
  const [ready, setReady] = useState(() => {
    // Already loaded in a previous render
    return typeof window !== 'undefined' && !!customElements.get('openai-chatkit');
  });

  useEffect(() => {
    if (customElements.get('openai-chatkit')) {
      setReady(true);
      return;
    }

    // Check if script tag already exists
    const existing = document.querySelector(`script[src="${CHATKIT_CDN}"]`);
    if (existing) {
      customElements.whenDefined('openai-chatkit').then(() => setReady(true));
      return;
    }

    const script = document.createElement('script');
    script.src = CHATKIT_CDN;
    script.async = true;
    script.onload = () => {
      customElements.whenDefined('openai-chatkit').then(() => setReady(true));
    };
    script.onerror = () => {
      console.error('Failed to load ChatKit web component from CDN');
    };
    document.head.appendChild(script);
  }, []);

  return ready;
}

function ChatWidgetInner() {
  const { accessToken } = useAuth();
  const { colorMode } = useColorMode();
  const [isOpen, setIsOpen] = useState(false);
  const scriptReady = useLoadChatKitScript();

  // Custom fetch that injects Authorization header
  const customFetch = useCallback(
    (url: RequestInfo | URL, init?: RequestInit) => {
      const headers = new Headers(init?.headers);
      if (accessToken) {
        headers.set('Authorization', `Bearer ${accessToken}`);
      }
      return fetch(url, { ...init, headers });
    },
    [accessToken],
  );

  return (
    <>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
        title={isOpen ? 'Close chat' : 'Ask the textbook'}
      >
        {isOpen ? '\u2715' : '\uD83D\uDCAC'}
      </button>

      {isOpen && (
        <div className={`${styles.chatContainer} ${colorMode === 'dark' ? styles.dark : styles.light}`}>
          <div className={styles.chatHeader}>
            <span className={styles.chatTitle}>Textbook Assistant</span>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              &times;
            </button>
          </div>
          <div className={styles.chatBody}>
            {scriptReady ? (
              <ChatKitEmbed
                apiUrl={`${API_URL}/chatkit`}
                customFetch={customFetch}
                colorMode={colorMode}
              />
            ) : (
              <div className={styles.loading}>Loading chat...</div>
            )}
          </div>
        </div>
      )}
    </>
  );
}

interface ChatKitEmbedProps {
  apiUrl: string;
  customFetch: typeof fetch;
  colorMode: string;
}

function ChatKitEmbed({ apiUrl, customFetch, colorMode }: ChatKitEmbedProps) {
  const { control } = useChatKit({
    api: {
      url: apiUrl,
      fetch: customFetch,
      domainKey: 'local-dev',
    },
    history: {
      enabled: true,
    },
    theme: colorMode === 'dark' ? 'dark' : 'light',
  });

  return (
    <ChatKit
      control={control}
      style={{ width: '100%', height: '100%' }}
    />
  );
}

export default function ChatWidget(): JSX.Element {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ChatWidgetInner />}
    </BrowserOnly>
  );
}
