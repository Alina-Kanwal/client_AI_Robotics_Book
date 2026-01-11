/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro/index', // Introduction page
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'doc',
          id: 'module1/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        {
          type: 'doc',
          id: 'module2/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        {
          type: 'doc',
          id: 'module3/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        {
          type: 'doc',
          id: 'module4/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: [
        {
          type: 'doc',
          id: 'capstone/index',
        },
      ],
    },
    {
      type: 'doc',
      id: 'conclusion/index', // Conclusion page
    },
  ],
};

export default sidebars;