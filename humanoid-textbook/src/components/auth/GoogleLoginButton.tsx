import { API_URL } from '@site/src/config';
import styles from './auth.module.css';


interface GoogleLoginButtonProps {
  disabled?: boolean;
}

export default function GoogleLoginButton({ disabled = false }: GoogleLoginButtonProps): JSX.Element {
  const handleGoogleLogin = () => {
    window.location.href = `${API_URL}/api/auth/google/login`;
  };

  return (
    <button
      className={styles.secondaryButton}
      onClick={handleGoogleLogin}
      disabled={disabled}
    >
      🔵 Login with Google
    </button>
  );
}
