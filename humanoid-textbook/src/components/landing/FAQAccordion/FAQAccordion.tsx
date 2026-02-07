import type {ReactNode} from 'react';
import styles from './FAQAccordion.module.css';

type FAQAccordionProps = {
  question: string;
  answer: string;
};

export default function FAQAccordion({
  question,
  answer,
}: FAQAccordionProps): ReactNode {
  return (
    <details className={styles.details}>
      <summary className={styles.summary}>{question}</summary>
      <p className={styles.answer}>{answer}</p>
    </details>
  );
}
