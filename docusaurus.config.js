// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

// import { themes as prismThemes } from 'prism-react-renderer';

import darkCode from './src/utils/codeDark.ts';
import lightCode from './src/utils/codeLight.ts';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'EZ-Template',
  tagline: 'robotics is ez',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://ez-robotics.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/EZ-Template/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'EZ-Robotics', // Usually your GitHub org/user name.
  projectName: 'EZ-Template', // Usually your repo name.
  trailingSlash: false,
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

          // For when 3.0 is actually released
          lastVersion: 'current',
          versions: {
            current: {
              label: '3.2.1',
              banner: 'none',
            },
            '2.x': {
              label: '2.x',
              banner: 'unmaintained',
            },
          },

          /*
          // While 3.0 isn't released
          lastVersion: '2.x',
          versions: {
            current: {
              label: '3.0.0-pre-release',
              banner: 'unreleased',
            },
            '2.x': {
              label: '2.x',
              banner: 'none',
            },
          },
          */

          routeBasePath: '/', // url
          path: './ez-template-docs', // file path
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: ({ versionDocsDirPath, docPath }) =>
            `https://github.com/ez-robotics/EZ-Template/tree/website/${versionDocsDirPath}/${docPath}`,
        },

        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],


  ],


  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'community',
        path: 'community', // file path
        routeBasePath: 'community', // url
        sidebarPath: './sidebarsCommunity.js',
        // ... other options
      },
    ],
  ],





  themeConfig:




    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({

      algolia: {
        // The application ID provided by Algolia
        appId: 'X63SPMQ1B4',

        // Public API key: it is safe to commit it
        apiKey: '75492b6c37c107a66c7ad93a8242d71e',

        indexName: 'ez-roboticsio',

        // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
        externalUrlRegex: 'external\\.com|domain\\.com',

        // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
        replaceSearchResultPathname: {
          from: '/docs/', // or as RegExp: /\/docs\//
          to: '/',
        },

        // Optional: Algolia search parameters
        searchParameters: {},

        // Optional: path for search page that enabled by default (`false` to disable it)
        searchPagePath: 'search',

        // Optional: whether the insights feature is enabled or not on Docsearch (`false` by default)
        insights: false,

        //... other Algolia params
      },



      announcementBar: {
        id: 'new_version',
        content:
          '🥳 <a target="_blank" rel="noopener noreferrer" href="https://github.com/EZ-Robotics/EZ-Template/releases/tag/v3.2.1">EZ-Template v3.2.1</a> is out! 🥳',
        backgroundColor: '#FDFD96',
        textColor: '#000000',
        isCloseable: true,
      },

      colorMode: {
        defaultMode: 'dark',
        disableSwitch: true,
        respectPrefersColorScheme: false,
      },

      // Replace with your project's social card
      image: 'img/embed.png',
      navbar: {
        title: 'EZ-Template',
        items: [
          {
            type: 'docsVersionDropdown',
            position: 'right',
            dropdownItemsAfter: [{ to: '/versions', label: 'Versions' }],
            dropdownActiveClassDisabled: true,
          },
          // { to: '/', label: 'EZ-Template', position: 'left' },
          { to: '/category/tutorials', label: 'Tutorials', position: 'left' },
          { to: '/category/docs', label: 'Docs', position: 'left' },
          { to: '/community/category/showcase', label: 'Showcase', position: 'left' },
          { to: '/community/support', label: 'Support', position: 'left' },
          { href: 'https://www.roboticsisez.com/', label: 'robotics is ez', position: 'right' },
          { href: 'https://github.com/EZ-Robotics/EZ-Template', label: 'GitHub', position: 'right' },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'EZ-Template',
                to: '/',
              },
              {
                label: 'Tutorials',
                to: '/category/tutorials',
              },
              {
                label: 'Docs',
                to: '/category/docs',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discord.gg/EHjXBcK2Gy',
              },
              {
                label: 'Instagram',
                href: 'https://www.instagram.com/roboticsisez',
              },
              {
                label: 'Support',
                href: '/community/support',
              },
              {
                label: 'Showcase',
                href: '/community/category/showcase',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'robotics is ez',
                href: 'https://www.roboticsisez.com/',
              },
              {
                label: 'Changelog',
                href: '/versions',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/EZ-Robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} robotics is ez - built with docusaurus`,
      },
      prism: {
        theme: lightCode,
        darkTheme: darkCode,
      },
    }),
};

export default config;
