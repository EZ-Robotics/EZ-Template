/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/**  @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docs: [
    'support',

    {
      type: 'category',
      label: 'Showcase',
      link: {
        type: 'generated-index',
      },
      collapsed: false,
      collapsible: true,
      items: [

        'showcase/hall_of_fame',
        // 'showcase/highstakes',  // Bring this back once autos actually exist
        'showcase/overunder',
        'showcase/spinup',
        'showcase/tippingpoint',
        'showcase/changeup',
        'showcase/towertakeover',


      ],
    },
  ],
};
export default sidebars;