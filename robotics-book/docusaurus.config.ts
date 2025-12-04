import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course on building intelligent physical systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://wasia93.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/docusaurus-robotics-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Wasia93', // Usually your GitHub org/user name.
  projectName: 'docusaurus-robotics-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Internationalization configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'sd', 'pa', 'ps', 'fr', 'zh'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'Urdu (اردو)',
        direction: 'rtl',
        htmlLang: 'ur',
      },
      sd: {
        label: 'Sindhi (سنڌي)',
        direction: 'rtl',
        htmlLang: 'sd',
      },
      pa: {
        label: 'Punjabi (ਪੰਜਾਬੀ)',
        direction: 'ltr',
        htmlLang: 'pa',
      },
      ps: {
        label: 'Pashto (پښتو)',
        direction: 'rtl',
        htmlLang: 'ps',
      },
      fr: {
        label: 'Français',
        direction: 'ltr',
        htmlLang: 'fr',
      },
      zh: {
        label: '中文',
        direction: 'ltr',
        htmlLang: 'zh-CN',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Make docs the home page
          showLastUpdateTime: true,
          // Remove edit URL for now
          // editUrl: 'https://github.com/your-org/robotics-book/edit/main/',
        },
        blog: false, // Disable blog for now, focus on course content
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Robotics Course Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/Wasia93/docusaurus-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course Content',
          items: [
            {
              label: 'Introduction',
              to: '/foundations/intro-physical-ai',
            },
            {
              label: 'ROS 2 Module',
              to: '/module1-ros2/ros2-overview',
            },
            {
              label: 'Isaac Platform',
              to: '/module3-isaac/isaac-sim',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/Wasia93/docusaurus-robotics-book',
            },
            {
              label: 'Discussion Forum',
              href: 'https://github.com/Wasia93/docusaurus-robotics-book/discussions',
            },
          ],
        },
        {
          title: 'About',
          items: [
            {
              label: 'About This Course',
              to: '/',
            },
            {
              label: 'License',
              href: 'https://github.com/Wasia93/docusaurus-robotics-book/blob/main/LICENSE',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash'],
    },
    algolia: {
      // Search configuration (configure when you have Algolia account)
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_API_KEY',
      indexName: 'robotics-book',
      contextualSearch: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
