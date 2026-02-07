import Link from '@docusaurus/Link';
import Card from '@site/src/components/landing/Card/Card';
import FAQAccordion from '@site/src/components/landing/FAQAccordion/FAQAccordion';
import NumberedOutcome from '@site/src/components/landing/NumberedOutcome/NumberedOutcome';
import StatItem from '@site/src/components/landing/StatItem/StatItem';
import TierBadge from '@site/src/components/landing/TierBadge/TierBadge';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import type { ReactNode } from 'react';

import styles from './index.module.css';

// ── Data Definitions ──────────────────────────────────────────────────

type Stat = {number: string; label: string};
const STATS: Stat[] = [
  {number: '13+', label: 'Weeks'},
  {number: '4', label: 'Modules'},
  {number: '40', label: 'Lessons'},
  {number: '4', label: 'Hardware Tiers'},
];

type ValueCard = {title: string; description: string};
const VALUE_CARDS: ValueCard[] = [
  {
    title: 'The $150B Opportunity',
    description:
      'Goldman Sachs projects the humanoid robot market will reach $154 billion by 2035. Companies including Tesla, Figure, NVIDIA, and Boston Dynamics are racing to deploy humanoid workers. Engineers who understand Physical AI will shape this industry.',
  },
  {
    title: 'Where Software Meets Hardware',
    description:
      'Large Language Models can reason and plan. Physical AI gives them a body. This course teaches the complete stack: from ROS 2 middleware and physics simulation to perception pipelines and voice-controlled robot actions.',
  },
  {
    title: 'Learn by Building',
    description:
      'Every module produces working artifacts. You will build ROS 2 packages, simulate robots in Gazebo, train perception models with NVIDIA Isaac, and deploy a voice-controlled humanoid in your capstone project.',
  },
];

type ModuleData = {
  number: number;
  title: string;
  focus: string;
  weeks: string;
  tiers: number[];
  status: 'Available' | 'Coming Soon';
  href?: string;
};
const MODULES: ModuleData[] = [
  {
    number: 1,
    title: 'The Robotic Nervous System',
    focus: 'ROS 2 Middleware',
    weeks: 'Weeks 1-5',
    tiers: [1, 2],
    status: 'Available',
    href: '/docs/module-1-ros2',
  },
  {
    number: 2,
    title: 'The Digital Twin',
    focus: 'Gazebo & Unity Simulation',
    weeks: 'Weeks 6-7',
    tiers: [2],
    status: 'Coming Soon',
  },
  {
    number: 3,
    title: 'The AI-Robot Brain',
    focus: 'NVIDIA Isaac Platform',
    weeks: 'Weeks 8-10',
    tiers: [2, 3],
    status: 'Coming Soon',
  },
  {
    number: 4,
    title: 'Vision-Language-Action',
    focus: 'LLMs Meet Robotics',
    weeks: 'Weeks 11-13',
    tiers: [3, 4],
    status: 'Coming Soon',
  },
];

type TierData = {
  tier: number;
  name: string;
  equipment: string;
  cost: string;
  description: string;
};
const TIERS: TierData[] = [
  {
    tier: 1,
    name: 'Cloud Explorer',
    equipment: 'Any laptop with a browser',
    cost: '$0',
    description:
      'Follow all conceptual lessons, run ROS 2 through cloud environments like Foxglove Studio and The Construct, and complete browser-based exercises.',
  },
  {
    tier: 2,
    name: 'Local Developer',
    equipment: 'RTX GPU + Ubuntu 22.04',
    cost: '$1,500-3,000',
    description:
      'Run ROS 2 Humble natively, launch Gazebo simulations, train models locally, and build full development workstations.',
  },
  {
    tier: 3,
    name: 'Edge Deployer',
    equipment: 'Jetson Orin Nano + RealSense Camera',
    cost: '~$700',
    description:
      'Deploy perception models to edge hardware, run real-time inference, and connect physical sensors for SLAM and navigation.',
  },
  {
    tier: 4,
    name: 'Robot Operator',
    equipment: 'Unitree Go2 or G1 Humanoid',
    cost: '$3,000-16,000+',
    description:
      'Execute sim-to-real transfer, control physical robots with ROS 2 nodes, and complete the capstone with a walking humanoid.',
  },
];

const OUTCOMES: string[] = [
  'Explain Physical AI principles and the transition from digital to embodied intelligence',
  'Build and deploy ROS 2 nodes, services, and actions for robotic control systems',
  'Simulate humanoid robots in Gazebo and Unity with realistic physics and sensors',
  'Develop perception pipelines using NVIDIA Isaac for visual SLAM and navigation',
  'Design humanoid robot kinematics for bipedal locomotion and object manipulation',
  'Integrate large language models with ROS 2 for voice-controlled robot actions',
];

type FAQItem = {question: string; answer: string};
const FAQS: FAQItem[] = [
  {
    question: 'Do I need a powerful GPU to take this course?',
    answer:
      'No. Every lesson includes a Tier 1 path that works with any laptop and a web browser. You can complete the conceptual content and many hands-on exercises using cloud-based tools like Foxglove Studio and The Construct. A local GPU becomes valuable in Module 2 for physics simulation, but it is not required to start learning.',
  },
  {
    question: 'What programming experience do I need?',
    answer:
      'You need working knowledge of Python, including functions, classes, and basic data structures. Experience with Linux command-line tools (navigating directories, running scripts) will help. No prior robotics experience is required.',
  },
  {
    question:
      'Is this course only theoretical, or will I build real projects?',
    answer:
      'You will build working projects in every module. Module 1 produces ROS 2 packages. Module 2 creates robot simulations. Module 3 builds perception pipelines. The capstone in Module 4 produces a voice-controlled humanoid robot that navigates obstacles and manipulates objects.',
  },
  {
    question: 'Which ROS 2 distribution does this course use?',
    answer:
      'This course targets ROS 2 Humble Hawksbill, the current Long-Term Support release running on Ubuntu 22.04. Code examples are also compatible with ROS 2 Iron where noted.',
  },
  {
    question: 'Can I use macOS or Windows instead of Ubuntu?',
    answer:
      'ROS 2 runs natively on Ubuntu 22.04. For macOS and Windows users, the recommended approach is a dual-boot setup or a virtual machine. Docker-based workflows are possible but add complexity. The Tier 1 cloud path works on any operating system.',
  },
  {
    question: 'How long does the full course take to complete?',
    answer:
      'The course spans 13 weeks with approximately 8-10 hours of study per week. Each lesson includes a duration estimate so you can plan your sessions. You can study at your own pace, but the weekly structure provides a recommended cadence.',
  },
];

// ── Sections ──────────────────────────────────────────────────────────

function HeroSection(): ReactNode {
  return (
    <header className={clsx('hero', styles.hero)}>
      <div className={clsx('container', styles.heroInner)}>
        <span className={styles.eyebrow}>PHYSICAL AI &amp; HUMANOID ROBOTICS</span>
        <h1 className={clsx('hero__title', styles.heroTitle)}>
          From ChatGPT to Walking Robots
        </h1>
        <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
          Learn to build AI systems that move through the physical world. This
          13-week course takes you from ROS 2 fundamentals through simulation
          and perception to deploying humanoid robots with voice-controlled
          intelligence.
        </p>
        <div className={styles.heroCtas}>
          <Link
            className="button button--primary button--lg"
            to="/docs/module-1-ros2">
            Start Learning
          </Link>
          <a
            className="button button--secondary button--lg"
            href="#modules">
            Explore Modules
          </a>
        </div>
      </div>
    </header>
  );
}

function StatsBar(): ReactNode {
  return (
    <section className={styles.statsBar}>
      <div className={clsx('container', styles.statsContainer)}>
        {STATS.map((stat) => (
          <StatItem key={stat.label} number={stat.number} label={stat.label} />
        ))}
      </div>
    </section>
  );
}

function WhyPhysicalAI(): ReactNode {
  return (
    <section className={styles.section}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why Physical AI?</h2>
        <p className={styles.sectionSubtitle}>
          The next frontier of artificial intelligence extends beyond screens
          into the physical world.
        </p>
        <div className={styles.cardGrid3}>
          {VALUE_CARDS.map((card) => (
            <Card key={card.title} title={card.title}>
              <p>{card.description}</p>
            </Card>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulePreview(): ReactNode {
  return (
    <section id="modules" className={styles.sectionAlt}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <div className={styles.cardGrid2}>
          {MODULES.map((mod) => (
            <Card
              key={mod.number}
              title={`Module ${mod.number}: ${mod.title}`}
              dimmed={mod.status === 'Coming Soon'}
              href={mod.href}>
              <p className={styles.moduleFocus}>{mod.focus}</p>
              <p className={styles.moduleWeeks}>{mod.weeks}</p>
              <div className={styles.moduleMeta}>
                <div>
                  {mod.tiers.map((t) => (
                    <TierBadge key={t} tier={t} />
                  ))}
                </div>
                <span
                  className={clsx(
                    styles.statusPill,
                    mod.status === 'Available'
                      ? styles.statusAvailable
                      : styles.statusSoon,
                  )}>
                  {mod.status}
                </span>
              </div>
            </Card>
          ))}
        </div>
      </div>
    </section>
  );
}

function ChooseYourPath(): ReactNode {
  return (
    <section className={styles.section}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Choose Your Path</h2>
        <p className={styles.sectionSubtitle}>
          This course works at every equipment level. Start where you are and
          scale up when ready.
        </p>
        <div className={styles.cardGrid4}>
          {TIERS.map((tier) => (
            <Card
              key={tier.tier}
              title={`Tier ${tier.tier}: ${tier.name}`}
              highlighted={tier.tier === 1}>
              {tier.tier === 1 && (
                <TierBadge tier={1} label="Recommended Start" />
              )}
              <p className={styles.tierEquipment}>{tier.equipment}</p>
              <p className={styles.tierCost}>{tier.cost}</p>
              <p>{tier.description}</p>
            </Card>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningOutcomes(): ReactNode {
  return (
    <section className={styles.sectionAlt}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What You Will Be Able to Do</h2>
        <p className={styles.sectionSubtitle}>
          By the end of this course, you will have the skills to build
          intelligent physical systems.
        </p>
        <div className={styles.outcomesGrid}>
          {OUTCOMES.map((text, i) => (
            <NumberedOutcome key={i} number={i + 1} text={text} />
          ))}
        </div>
      </div>
    </section>
  );
}

function FAQSection(): ReactNode {
  return (
    <section className={styles.section}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Frequently Asked Questions</h2>
        <div className={styles.faqList}>
          {FAQS.map((faq) => (
            <FAQAccordion
              key={faq.question}
              question={faq.question}
              answer={faq.answer}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

// ── Page Layout ───────────────────────────────────────────────────────

export default function Home(): ReactNode {
  return (
    <Layout
      title="From ChatGPT to Walking Robots"
      description="A 13-week course on Physical AI and Humanoid Robotics — from ROS 2 fundamentals to deploying voice-controlled humanoid robots.">
      <HeroSection />
      <StatsBar />
      <main>
        <WhyPhysicalAI />
        <ModulePreview />
        <ChooseYourPath />
        <LearningOutcomes />
        <FAQSection />
      </main>
    </Layout>
  );
}
