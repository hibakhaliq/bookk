import React from 'react';
import clsx from 'clsx';
import styles from './Hero.module.css';

export default function Hero() {
  return (
    <header className={clsx(styles.hero)}>
      <div className={styles.container}>
        <h1 className={styles.title}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={styles.subtitle}>
          Learn Robotics, ROS 2, Gazebo, and AI Integration step by step
        </p>
        <div className={styles.buttons}>
          <a className={styles.button} href="#get-started">
            Get Started
          </a>
          <a className={clsx(styles.button, styles.buttonSecondary)} href="#docs">
            Explore Docs
          </a>
        </div>
      </div>
    </header>
  );
}
