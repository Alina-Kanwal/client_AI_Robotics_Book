/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro', // This will be the homepage
    },
    {
      type: 'category',
      label: 'Chapters',
      items: [
        {
          type: 'doc',
          id: 'chapters/chapter1',
        },
        {
          type: 'doc',
          id: 'chapters/chapter2',
        },
        {
          type: 'doc',
          id: 'chapters/chapter3',
        },
      ],
    },
    {
      type: 'doc',
      id: 'conclusion',
    },
  ],
};

export default sidebars;