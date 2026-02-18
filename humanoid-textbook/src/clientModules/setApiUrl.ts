/**
 * Docusaurus client module: injects build-time API_URL into
 * window.__DOCUSAURUS_API_URL__ so src/config.ts picks it up at runtime.
 *
 * Values come from docusaurus.config.ts → customFields, which are populated
 * from environment variables (API_URL, FRONTEND_URL) at build time.
 */
import siteConfig from '@generated/docusaurus.config';

declare global {
  interface Window {
    __DOCUSAURUS_API_URL__: string;
    __DOCUSAURUS_FRONTEND_URL__: string;
  }
}

const fields = (siteConfig.customFields ?? {}) as Record<string, string>;

if (typeof window !== 'undefined') {
  if (fields.apiUrl) {
    window.__DOCUSAURUS_API_URL__ = fields.apiUrl;
  }
  if (fields.frontendUrl) {
    window.__DOCUSAURUS_FRONTEND_URL__ = fields.frontendUrl;
  }
}
