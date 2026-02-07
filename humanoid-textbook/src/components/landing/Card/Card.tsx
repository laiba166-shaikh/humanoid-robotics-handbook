import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './Card.module.css';

type CardProps = {
  title: string;
  children: ReactNode;
  highlighted?: boolean;
  dimmed?: boolean;
  href?: string;
  className?: string;
};

export default function Card({
  title,
  children,
  highlighted,
  dimmed,
  href,
  className,
}: CardProps): ReactNode {
  const cardContent = (
    <div
      className={clsx(
        'card',
        styles.card,
        highlighted && styles.highlighted,
        dimmed && styles.dimmed,
        className,
      )}>
      <h3 className={styles.title}>{title}</h3>
      <div className={styles.body}>{children}</div>
    </div>
  );

  if (href && !dimmed) {
    return (
      <Link to={href} className={styles.link}>
        {cardContent}
      </Link>
    );
  }

  return cardContent;
}
