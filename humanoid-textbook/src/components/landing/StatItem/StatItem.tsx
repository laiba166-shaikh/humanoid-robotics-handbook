import type {ReactNode} from 'react';
import styles from './StatItem.module.css';

type StatItemProps = {
  number: string;
  label: string;
};

export default function StatItem({number, label}: StatItemProps): ReactNode {
  return (
    <div className={styles.stat}>
      <span className={styles.number}>{number}</span>
      <span className={styles.label}>{label}</span>
    </div>
  );
}
