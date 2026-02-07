import type {ReactNode} from 'react';
import clsx from 'clsx';
import styles from './TierBadge.module.css';

type TierBadgeProps = {
  tier: number;
  label?: string;
};

export default function TierBadge({tier, label}: TierBadgeProps): ReactNode {
  return (
    <span className={clsx('tier-badge', `tier-badge--${tier}`, styles.badge)}>
      {label ?? `Tier ${tier}`}
    </span>
  );
}
