import { useHistory, useLocation } from '@docusaurus/router';
import { useAuth } from '@site/src/components/auth/AuthProvider';
import GoogleLoginButton from '@site/src/components/auth/GoogleLoginButton';
import { FRONTEND_URL } from '@site/src/config';
import React, { useEffect, useState } from 'react';
import styles from '../../components/auth/auth.module.css';

interface LoginFormData {
  email: string;
  password: string;
}

interface LoginErrors {
  email?: string;
  password?: string;
  general?: string;
}

export default function LoginPage(): JSX.Element {
  const history = useHistory();
  const location = useLocation();
  const { login, isLoading } = useAuth();

  const [formData, setFormData] = useState<LoginFormData>({
    email: '',
    password: '',
  });
  const [errors, setErrors] = useState<LoginErrors>({});
  const [showPassword, setShowPassword] = useState(false);
  const [oauthError, setOauthError] = useState<string | null>(null);

  // Check for OAuth error in query params
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    if (params.get('error') === 'oauth_failed') {
      setOauthError('Google login was cancelled. Try again or use email.');
    }
  }, [location.search]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    // Clear error for this field when user starts typing
    if (errors[name as keyof LoginErrors]) {
      setErrors((prev) => ({ ...prev, [name]: undefined }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setErrors({});
    setOauthError(null);

    try {
      await login(formData.email, formData.password);
      // Redirect to docs landing page on success
      window.location.href = `${FRONTEND_URL}/docs`;
    } catch (error) {
      setFormData({ email: '', password: '' });
      setErrors({
        general:
          error instanceof Error
            ? error.message
            : 'Login failed. Please try again.',
      });
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1>Log In</h1>
        <p className={styles.subtitle}>Access the handbook</p>

        {oauthError && <div className={styles.errorBanner}>{oauthError}</div>}
        {errors.general && (
          <div className={styles.errorBanner}>{errors.general}</div>
        )}

        <form onSubmit={handleSubmit}>
          <div className={styles.formGroup}>
            <label htmlFor='email'>Email</label>
            <input
              id='email'
              name='email'
              type='email'
              value={formData.email}
              onChange={handleChange}
              placeholder='you@example.com'
              required
              disabled={isLoading}
            />
            {errors.email && (
              <span className={styles.errorText}>{errors.email}</span>
            )}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor='password'>Password</label>
            <div className={styles.passwordWrapper}>
              <input
                id='password'
                name='password'
                type={showPassword ? 'text' : 'password'}
                value={formData.password}
                onChange={handleChange}
                placeholder='Your password'
                required
                disabled={isLoading}
              />
              <button
                type='button'
                className={styles.togglePassword}
                onClick={() => setShowPassword(!showPassword)}
                disabled={isLoading}
              >
                {showPassword ? '👁️' : '👁️‍🗨️'}
              </button>
            </div>
            {errors.password && (
              <span className={styles.errorText}>{errors.password}</span>
            )}
          </div>

          <button
            type='submit'
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Logging in...' : 'Log In'}
          </button>
        </form>

        <div className={styles.divider}>
          <span>or</span>
        </div>

        <GoogleLoginButton disabled={isLoading} />

        <p className={styles.switchAuth}>
          Don't have an account?{' '}
          <a href={`${FRONTEND_URL}/auth/signup`}>Sign up</a>
        </p>
      </div>
    </div>
  );
}
