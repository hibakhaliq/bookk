import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1 - Introduction to Robotics',
      items: [
        'chapter-1/index',
        'chapter-1/purpose',
        'chapter-1/technical-integration',
        'chapter-1/assignments',
        'chapter-1/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2 - ROS 2 Fundamentals',
      items: [
        'chapter-2/index',
        'chapter-2/purpose',
        'chapter-2/technical-integration',
        'chapter-2/assignments',
        'chapter-2/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3 - Gazebo Simulation',
      items: [
        'chapter-3/index',
        'chapter-3/purpose',
        'chapter-3/technical-integration',
        'chapter-3/assignments',
        'chapter-3/assessment'
      ],
    },
    // Add more chapters as needed
  ],
};

export default sidebars;
