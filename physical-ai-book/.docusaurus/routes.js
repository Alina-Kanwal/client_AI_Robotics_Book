import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '378'),
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
    path: '*',
    component: ComponentCreator('*'),
  },
];
