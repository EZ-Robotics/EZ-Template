// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'EZ Template',
  tagline: '',
  url: 'https://ez-robotics.github.io',
  baseUrl: '/EZ-Template/',
  projectName: 'EZ-Template',
  organizationName: 'EZ-Robotics',
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/pog.ico',
  deploymentBranch: 'site',
  plugins: [
    [
      require.resolve("@cmfcmf/docusaurus-search-local"),
      {
        // whether to index docs pages
        indexDocs: true,
      
        // Whether to also index the titles of the parent categories in the sidebar of a doc page.
        // 0 disables this feature.
        // 1 indexes the direct parent category in the sidebar of a doc page
        // 2 indexes up to two nested parent categories of a doc page
        // 3...
        //
        // Do _not_ use Infinity, the value must be a JSON-serializable integer.
        indexDocSidebarParentCategories: 2,
      
      
        // whether to index static pages
        // /404.html is never indexed
        indexPages: false,
      
        // language of your documentation, see next section
        language: "en",

        style: undefined,
        // style: require.resolve( './src/css/custom.css' ),
        lunr: {
          // When indexing your documents, their content is split into "tokens".
          // Text entered into the search box is also tokenized.
          // This setting configures the separator used to determine where to split the text into tokens.
          // By default, it splits the text at whitespace and dashes.
          //
          // Note: Does not work for "ja" and "th" languages, since these use a different tokenizer.
          tokenizerSeparator: /[\s\-]+/,
          // https://lunrjs.com/guides/customising.html#similarity-tuning
          //
          // This parameter controls the importance given to the length of a document and its fields. This
          // value must be between 0 and 1, and by default it has a value of 0.75. Reducing this value
          // reduces the effect of different length documents on a term’s importance to that document.
          b: 0.75,
          // This controls how quickly the boost given by a common word reaches saturation. Increasing it
          // will slow down the rate of saturation and lower values result in quicker saturation. The
          // default value is 1.2. If the collection of documents being indexed have high occurrences
          // of words that are not covered by a stop word filter, these words can quickly dominate any
          // similarity calculation. In these cases, this value can be reduced to get more balanced results.
          k1: .6,
          // By default, we rank pages where the search term appears in the title higher than pages where
          // the search term appears in just the text. This is done by "boosting" title matches with a
          // higher value than content matches. The concrete boosting behavior can be controlled by changing
          // the following settings.
          titleBoost: 100,
          contentBoost: 1,
          parentCategoriesBoost: 8, // Only used when indexDocSidebarParentCategories > 0
        }

      }
    ],
  ],


  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          },

        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      metadata: [{name: 'keywords', content: 'EZ-template, VRC, VEX, Robotics, VEX-Robotics, library'}],
      navbar: {
        title: 'EZ Template',
        logo: {
          alt: 'EZ',
          src: 'img/logo.png',
        },
        items: [
          {to: '/category/tutorials', label: 'Tutorials', position: 'left'},
          
          {to: '/category/docs', label: 'Docs', position: 'left'},
          {to: '/category/releases', label: 'Releases', position: 'left'},

          {
            href: 'https://github.com/EZ-Robotics/EZ-Template',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
                  items: [
                    {
                      label: 'Tutorial',
                      to: '/category/tutorials',
                    },
                    {
                      label: 'Docs',
                      to: '/category/docs',
                    },
                    {
                      label: 'Releases',
                      to: '/category/releases',
                    },                    
                    
                  ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Vex Robotics Discord',
                href: 'https://discord.com/invite/vrc',
              },
              {
                label: 'roboticsisez@gmail.com',
                href: 'mailto:roboticsisez@gmail.com'

              }
            ],
          },
          
          // ... other links
        ],

        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
