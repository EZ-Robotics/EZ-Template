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
    'Introduction',
    {
      type: 'category',
      label: 'Tutorials',
      link: {
        type: 'generated-index',
      },
      collapsed: false,
      collapsible: true,
      items: [
        'tutorials/installation',
        'tutorials/using_auton_selector',
        'tutorials/autons',
        'tutorials/example_autons',
        'tutorials/tuning_constants',
        'tutorials/joystick_curve',
        'tutorials/activebrake',
        'tutorials/practice_mode',
        'tutorials/pid',
        'tutorials/pto_tutorial',
      ],
    },
    {
      type: 'category',
      label: 'Docs',
      link: {
        type: 'generated-index',
      },
      collapsed: false,
      collapsible: true,
      items: [
        'docs/constructor',
        'docs/auton_functions',
        'docs/user_control',
        'docs/auton_selector',
        'docs/set_and_get_drive',
        'docs/pid',
        'docs/pid_tuner',
        'docs/pto',
        'docs/slew',
        'docs/piston',
        'docs/util',
      ],
    },
    'migration',
  ],
};
export default sidebars;