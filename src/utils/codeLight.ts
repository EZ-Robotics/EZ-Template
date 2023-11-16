import {themes, type PrismTheme} from 'prism-react-renderer';

const baseTheme = themes.oneLight;

export default {
    ...baseTheme,
      styles: [
        ...baseTheme.styles,
        {
            types: ['comment'],
            style: {
              color: '#ad006b',
              fontStyle: "italic",
            },
          },
      ],
} satisfies PrismTheme;
