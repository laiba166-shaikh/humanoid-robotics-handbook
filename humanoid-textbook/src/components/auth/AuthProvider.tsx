import React, { createContext, useContext, useReducer, useEffect, ReactNode } from 'react';
import { authApi } from '../../services/authApi';

interface User {
  id: string;
  email: string;
  full_name?: string;
  auth_provider: string;
  is_active: boolean;
  created_at: string;
}

interface AuthContextType {
  user: User | null;
  accessToken: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signup: (email: string, password: string, full_name?: string) => Promise<void>;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  restoreSession: () => Promise<void>;
}

interface AuthState {
  user: User | null;
  accessToken: string | null;
  isLoading: boolean;
}

type AuthAction =
  | { type: 'SET_USER'; payload: User }
  | { type: 'SET_ACCESS_TOKEN'; payload: string }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'LOGOUT' };

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const initialState: AuthState = {
  user: null,
  accessToken: null,
  isLoading: false,
};

function authReducer(state: AuthState, action: AuthAction): AuthState {
  switch (action.type) {
    case 'SET_USER':
      return { ...state, user: action.payload };
    case 'SET_ACCESS_TOKEN':
      return { ...state, accessToken: action.payload };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'LOGOUT':
      return { ...state, user: null, accessToken: null };
    default:
      return state;
  }
}

export function AuthProvider({ children }: { children: ReactNode }): JSX.Element {
  const [state, dispatch] = useReducer(authReducer, initialState);

  // Restore session from localStorage on mount
  useEffect(() => {
    restoreSession();
  }, []);

  const restoreSession = async () => {
    dispatch({ type: 'SET_LOADING', payload: true });
    try {
      const refreshToken = localStorage.getItem('refresh_token');
      if (refreshToken) {
        const response = await authApi.refresh({ refresh_token: refreshToken });
        dispatch({ type: 'SET_ACCESS_TOKEN', payload: response.data.access_token });
        dispatch({ type: 'SET_USER', payload: response.data.user });
        authApi.setAccessToken(response.data.access_token);
        localStorage.setItem('refresh_token', response.data.refresh_token);
      }
    } catch (error) {
      // Session restore failed, clear tokens
      localStorage.removeItem('refresh_token');
    } finally {
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const signup = async (email: string, password: string, full_name?: string) => {
    dispatch({ type: 'SET_LOADING', payload: true });
    try {
      const response = await authApi.signup({ email, password, full_name });
      dispatch({ type: 'SET_ACCESS_TOKEN', payload: response.data.access_token });
      dispatch({ type: 'SET_USER', payload: response.data.user });
      authApi.setAccessToken(response.data.access_token);
      localStorage.setItem('refresh_token', response.data.refresh_token);
    } finally {
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const login = async (email: string, password: string) => {
    dispatch({ type: 'SET_LOADING', payload: true });
    try {
      const response = await authApi.login({ email, password });
      dispatch({ type: 'SET_ACCESS_TOKEN', payload: response.data.access_token });
      dispatch({ type: 'SET_USER', payload: response.data.user });
      authApi.setAccessToken(response.data.access_token);
      localStorage.setItem('refresh_token', response.data.refresh_token);
    } finally {
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const logout = async () => {
    dispatch({ type: 'SET_LOADING', payload: true });
    try {
      const refreshToken = localStorage.getItem('refresh_token');
      if (refreshToken) {
        await authApi.logout({ refresh_token: refreshToken });
      }
    } finally {
      dispatch({ type: 'LOGOUT' });
      localStorage.removeItem('refresh_token');
      authApi.setAccessToken(null);
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const value: AuthContextType = {
    user: state.user,
    accessToken: state.accessToken,
    isAuthenticated: !!state.user,
    isLoading: state.isLoading,
    signup,
    login,
    logout,
    restoreSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
