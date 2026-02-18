/**
 * Centralized app configuration.
 *
 * For local dev the defaults below work out of the box.
 * In production, set window.__DOCUSAURUS_API_URL__ (e.g. via a
 * script tag or Docusaurus plugin) to point at the deployed backend.
 */

export const API_URL: string =
  (typeof window !== 'undefined' && (window as any).__DOCUSAURUS_API_URL__) ||
  'https://humanoid-robotics-handbook-production.up.railway.app' ||
  'http://localhost:8000';

export const FRONTEND_URL: string =
  (typeof window !== 'undefined' &&
    (window as any).__DOCUSAURUS_FRONTEND_URL__) ||
  'https://laiba166-shaikh.github.io/humanoid-robotics-handbook' ||
  'http://localhost:3000/humanoid-robotics-handbook';
