import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '62a'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'df2'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '619'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '757'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', 'b71'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '166'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '43b'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '20a'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '9fe'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '235'),
            routes: [
              {
                path: '/docs/capstone/',
                component: ComponentCreator('/docs/capstone/', 'fd4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/conclusion/',
                component: ComponentCreator('/docs/conclusion/', '3e5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/',
                component: ComponentCreator('/docs/intro/', '015'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/',
                component: ComponentCreator('/docs/module1/', '998'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/',
                component: ComponentCreator('/docs/module2/', '424'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/',
                component: ComponentCreator('/docs/module3/', '214'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/',
                component: ComponentCreator('/docs/module4/', '72b'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'acf'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
