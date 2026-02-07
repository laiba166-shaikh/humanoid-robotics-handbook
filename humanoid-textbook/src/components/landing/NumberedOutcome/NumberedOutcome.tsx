import type {ReactNode} from 'react';
import styles from './NumberedOutcome.module.css';

type NumberedOutcomeProps = {
  number: number;
  text: string;
};

export default function NumberedOutcome({
  number,
  text,
}: NumberedOutcomeProps): ReactNode {
  return (
    <div className={styles.outcome}>
      <span className={styles.circle}>{number}</span>
      <p className={styles.text}>{text}</p>
    </div>
  );
}
