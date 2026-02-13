import { useAuth } from '@site/src/components/auth/AuthProvider';
import styles from './NavbarAuthItem.module.css';

export default function NavbarAuthItem(): JSX.Element {
  const { user, isAuthenticated, logout, isLoading } = useAuth();

  const handleLogout = async () => {
    await logout();
    window.location.href = '/';
  };

  if (isLoading) {
    return <div className={styles.navbarAuthItem}>Loading...</div>;
  }

  if (!isAuthenticated || !user) {
    return (
      <a href="auth/login" className={styles.loginLink}>
        Log In
      </a>
    );
  }

  return (
    <div className={styles.navbarAuthItem}>
      <span className={styles.userName}>{user.full_name || user.email}</span>
      <button
        className={styles.logoutButton}
        onClick={handleLogout}
        title="Log out"
      >
        Log Out
      </button>
    </div>
  );
}
