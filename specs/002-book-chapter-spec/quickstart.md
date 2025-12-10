# Quickstart: Setting Up the Robotics Book with Docusaurus

## Prerequisites
- Node.js 18 or higher
- npm or yarn package manager
- Git for version control

## Installation Steps

### 1. Create a New Docusaurus Project
```bash
npx create-docusaurus@latest book classic
cd book
```

### 2. Install Additional Dependencies
```bash
npm install --save-dev @docusaurus/module-type-aliases @docusaurus/types
npm install @docusaurus/preset-classic
```

### 3. Configure Docusaurus
Update `docusaurus.config.js` with book-specific settings:

```javascript
// docusaurus.config.js
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Robotics Book',
  tagline: 'A comprehensive guide to robotics with ROS 2, Gazebo, Isaac, VSLAM, and VLA',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-book-domain.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'robotics-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/your-book-repo/tree/main/',
        },
        blog: false, // Optional: disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Robotics Book',
        logo: {
          alt: 'Robotics Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/your-username/your-book-repo',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/robotics',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/your-book-repo',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['bash', 'python', 'cpp', 'json', 'yaml'],
      },
    }),
};

export default config;
```

### 4. Set Up Sidebar Navigation
Create `sidebars.js` to define the book's navigation structure:

```javascript
// sidebars.js
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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
    // Add more chapters as needed
  ],
};

export default sidebars;
```

### 5. Create Initial Book Structure
Create the basic folder structure for your book content:

```bash
mkdir -p docs/{chapter-1,chapter-2,chapter-3}
touch docs/intro.md
touch docs/chapter-1/{index.md,purpose.md,technical-integration.md,assignments.md,assessment.md}
touch docs/chapter-2/{index.md,purpose.md,technical-integration.md,assignments.md,assessment.md}
touch docs/chapter-3/{index.md,purpose.md,technical-integration.md,assignments.md,assessment.md}
```

### 6. Create Introduction Page
Create `docs/intro.md`:

```markdown
---
sidebar_position: 1
---

# Introduction to the Robotics Book

Welcome to the comprehensive robotics book that covers ROS 2, Gazebo, Isaac, VSLAM, and VLA technologies.

This book is designed to provide both theoretical knowledge and practical hands-on experience with modern robotics frameworks.

## What You'll Learn

- Fundamentals of robotics and autonomous systems
- Practical implementation using ROS 2
- Simulation techniques with Gazebo
- Advanced perception with Isaac and VSLAM
- Visual-language-action models with VLA

## How to Use This Book

Each chapter includes:
- Clear learning objectives
- Theoretical explanations
- Practical examples and code
- Assignments and in-class demonstrations
- Assessment materials

## Prerequisites

- Basic programming knowledge
- Understanding of linear algebra and calculus
- Familiarity with Linux command line
```

### 7. Start Development Server
```bash
npm run start
```

This will start a local development server at `http://localhost:3000` where you can preview your book.

## Adding Custom Components

To support robotics-specific content like interactive demos or technical diagrams, create custom React components in the `src/components/` directory:

```bash
mkdir -p src/components
```

Create a simple example component `src/components/InteractiveDemo/index.js`:

```javascript
import React from 'react';
import styles from './InteractiveDemo.module.css';

function InteractiveDemo({title, description, demoComponent}) {
  return (
    <div className={styles.interactiveDemo}>
      <h3>{title}</h3>
      <p>{description}</p>
      <div className={styles.demoContainer}>
        {demoComponent}
      </div>
    </div>
  );
}

export default InteractiveDemo;
```

## Deployment

To build and deploy your book:

```bash
npm run build
```

The static site will be generated in the `build/` directory and can be deployed to any static hosting service (GitHub Pages, Netlify, Vercel, etc.).

## Next Steps

1. Add your chapter content to the appropriate markdown files
2. Customize the theme and styling in `src/css/custom.css`
3. Add images and diagrams to the `static/img/` directory
4. Implement custom components for technical content
5. Set up continuous integration for automated builds