/**
 * Auth API client for communicating with the backend auth endpoints.
 * Handles token management and automatic token refresh on 401.
 */

import { API_URL } from '../config';

interface SignupPayload {
  email: string;
  password: string;
  full_name?: string;
}

interface LoginPayload {
  email: string;
  password: string;
}

interface RefreshPayload {
  refresh_token: string;
}

interface AuthResponse {
  success: boolean;
  data: {
    user: {
      id: string;
      email: string;
      full_name?: string;
      auth_provider: string;
      is_active: boolean;
      created_at: string;
    };
    access_token: string;
    refresh_token: string;
  };
  error?: string;
  meta: Record<string, unknown>;
}

interface UserResponse {
  success: boolean;
  data: {
    user: {
      id: string;
      email: string;
      full_name?: string;
      auth_provider: string;
      is_active: boolean;
      created_at: string;
    };
  };
  error?: string;
}

interface MessageResponse {
  success: boolean;
  data?: { message: string };
  error?: string;
}

class AuthApi {
  private accessToken: string | null = null;

  setAccessToken(token: string | null) {
    this.accessToken = token;
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {},
  ): Promise<T> {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers,
    };

    if (this.accessToken) {
      headers['Authorization'] = `Bearer ${this.accessToken}`;
    }

    const response = await fetch(`${API_URL}${endpoint}`, {
      ...options,
      headers,
    });

    if (response.status === 401 && endpoint !== '/api/auth/refresh') {
      // Try to refresh token
      const refreshToken = localStorage.getItem('refresh_token');
      if (refreshToken) {
        try {
          await this.refresh({ refresh_token: refreshToken });
          // Retry original request with new token
          return this.request<T>(endpoint, options);
        } catch (error) {
          // Refresh failed, clear tokens
          this.accessToken = null;
          localStorage.removeItem('refresh_token');
          throw error;
        }
      }
    }

    const data = await response.json();

    if (!response.ok) {
      // Backend returns errors as {detail: {message: "..."}} or {detail: "string"}
      const detail = data.detail;
      let message: string;
      if (typeof detail === 'object' && detail?.message) {
        message = detail.message;
      } else if (typeof detail === 'string') {
        message = detail;
      } else {
        message = data.error || 'Request failed. Please try again.';
      }
      throw new Error(message);
    }

    return data;
  }

  async signup(payload: SignupPayload): Promise<AuthResponse> {
    return this.request<AuthResponse>('/api/auth/signup', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
  }

  async login(payload: LoginPayload): Promise<AuthResponse> {
    return this.request<AuthResponse>('/api/auth/login', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
  }

  async refresh(payload: RefreshPayload): Promise<AuthResponse> {
    return this.request<AuthResponse>('/api/auth/refresh', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
  }

  async logout(payload: RefreshPayload): Promise<MessageResponse> {
    return this.request<MessageResponse>('/api/auth/logout', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
  }

  async getMe(): Promise<UserResponse> {
    return this.request<UserResponse>('/api/auth/me', {
      method: 'GET',
    });
  }

  getGoogleLoginUrl(): string {
    return `${API_URL}/api/auth/google/login`;
  }
}

export const authApi = new AuthApi();
