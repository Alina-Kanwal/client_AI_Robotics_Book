import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', 'b03'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '772'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '0ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '72e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', 'b66'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '42e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'dd4'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '4bf'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'd7e'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'f28'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'e52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
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
    path: '*',
    component: ComponentCreator('*'),
  },
];
