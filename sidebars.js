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

        // Getting Started
        {
          type: 'category',
          label: 'Getting Started',
          link: {
            type: 'generated-index',
          },
          collapsed: false,
          collapsible: true,
          items: [
            'tutorials/installation',
            'tutorials/upgrading', // this needs to get broken up
            'tutorials/using_auton_selector',
            'tutorials/autons',
            'tutorials/example_autons',
            'tutorials/intake_control'
          ],
        },

        // User Control
        {
          type: 'category',
          label: 'User Control',
          link: {
            type: 'generated-index',
          },
          collapsed: true,
          collapsible: true,
          items: [
            'tutorials/control_schemes',
            'tutorials/joystick_curve',
            'tutorials/activebrake',
            'tutorials/practice_mode',
          ],
        },


        // Tuning Constants
        {
          type: 'category',
          label: 'Tuning Constants',
          link: {
            type: 'generated-index',
          },
          collapsed: true,
          collapsible: true,
          items: [
            'tutorials/slew_constants',
            'tutorials/tuning_constants',
            'tutorials/tuning_exit_conditions',
            // gain scheduling?
          ],
        },

        /*
        // Using EZ PID
        {
          type: 'category',
          label: 'Using EZ PID',
          link: {
            type: 'generated-index',
          },
          collapsed: true,
          collapsible: true,
          items: [
            'tutorials/pid', // this needs to get broken up
          ],
        },

        // Using a PTO
        {
          type: 'category',
          label: 'Using a PTO',
          link: {
            type: 'generated-index',
          },
          collapsed: true,
          collapsible: true,
          items: [
            'tutorials/pto_tutorial',
          ],
        },
        */

        // Remove these when above is uncommented
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
      collapsed: true,
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