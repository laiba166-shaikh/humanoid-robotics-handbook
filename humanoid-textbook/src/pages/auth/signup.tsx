import { useHistory } from '@docusaurus/router';
import GoogleLoginButton from '@site/src/components/auth/GoogleLoginButton';
import { API_URL, FRONTEND_URL } from '@site/src/config';
import { Eye, EyeOff } from 'lucide-react';
import React, { useState } from 'react';
import styles from '../../components/auth/auth.module.css';


interface SignupFormData {
  email: string;
  password: string;
  full_name: string;
}

interface SignupErrors {
  email?: string;
  password?: string[];
  full_name?: string;
  general?: string;
}

export default function SignupPage(): JSX.Element {
  const history = useHistory();
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    full_name: '',
  });
  const [errors, setErrors] = useState<SignupErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    // Clear error for this field when user starts typing
    if (errors[name as keyof SignupErrors]) {
      setErrors(prev => ({ ...prev, [name]: undefined }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setErrors({});

    try {
      const response = await fetch(`${API_URL}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(formData),
      });

      const data = await response.json();

      if (!response.ok) {
        if (data.detail?.messages) {
          setErrors({ password: data.detail.messages });
        } else if (data.detail?.message) {
          setErrors({ general: data.detail.message });
        } else {
          setErrors({ general: 'Signup failed. Please try again.' });
        }
        setFormData({ email: '', password: '', full_name: '' });
        return;
      }

      // Store tokens
      localStorage.setItem('refresh_token', data.data.refresh_token);
      // Access token stored in AuthProvider context

      // Redirect to docs landing page
      window.location.href = `${FRONTEND_URL}/docs`;
    } catch (error) {
      setErrors({ general: 'An error occurred. Please try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1>Create Account</h1>
        <p className={styles.subtitle}>Sign up to access the handbook</p>

        {errors.general && <div className={styles.errorBanner}>{errors.general}</div>}

        <form onSubmit={handleSubmit}>
          <div className={styles.formGroup}>
            <label htmlFor="full_name">Full Name (Optional)</label>
            <input
              id="full_name"
              name="full_name"
              type="text"
              value={formData.full_name}
              onChange={handleChange}
              placeholder="Jane Doe"
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="email">Email</label>
            <input
              id="email"
              name="email"
              type="email"
              value={formData.email}
              onChange={handleChange}
              placeholder="you@example.com"
              required
              disabled={isLoading}
            />
            {errors.email && <span className={styles.errorText}>{errors.email}</span>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <div className={styles.passwordWrapper}>
              <input
                id="password"
                name="password"
                type={showPassword ? 'text' : 'password'}
                value={formData.password}
                onChange={handleChange}
                placeholder="StrongP@ss1"
                required
                disabled={isLoading}
              />
              <button
                type="button"
                className={styles.togglePassword}
                onClick={() => setShowPassword(!showPassword)}
                disabled={isLoading}
              >
                {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
              </button>
            </div>
            {errors.password && (
              <ul className={styles.errorList}>
                {errors.password.map((err, idx) => (
                  <li key={idx}>{err}</li>
                ))}
              </ul>
            )}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Creating account...' : 'Sign Up'}
          </button>
        </form>

        <div className={styles.divider}>
          <span>or</span>
        </div>

        <GoogleLoginButton disabled={isLoading} />

        <p className={styles.switchAuth}>
          Already have an account? <a href={`${FRONTEND_URL}/auth/login`}>Log in</a>
        </p>
      </div>
    </div>
  );
}
