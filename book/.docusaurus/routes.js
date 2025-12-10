import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/book/blog',
    component: ComponentCreator('/book/blog', '90b'),
    exact: true
  },
  {
    path: '/book/blog/archive',
    component: ComponentCreator('/book/blog/archive', 'c6a'),
    exact: true
  },
  {
    path: '/book/blog/authors',
    component: ComponentCreator('/book/blog/authors', '5a4'),
    exact: true
  },
  {
    path: '/book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/book/blog/authors/all-sebastien-lorber-articles', 'f39'),
    exact: true
  },
  {
    path: '/book/blog/authors/yangshun',
    component: ComponentCreator('/book/blog/authors/yangshun', 'b1b'),
    exact: true
  },
  {
    path: '/book/blog/first-blog-post',
    component: ComponentCreator('/book/blog/first-blog-post', 'b43'),
    exact: true
  },
  {
    path: '/book/blog/long-blog-post',
    component: ComponentCreator('/book/blog/long-blog-post', 'c9d'),
    exact: true
  },
  {
    path: '/book/blog/mdx-blog-post',
    component: ComponentCreator('/book/blog/mdx-blog-post', 'b01'),
    exact: true
  },
  {
    path: '/book/blog/tags',
    component: ComponentCreator('/book/blog/tags', 'ce3'),
    exact: true
  },
  {
    path: '/book/blog/tags/docusaurus',
    component: ComponentCreator('/book/blog/tags/docusaurus', '56a'),
    exact: true
  },
  {
    path: '/book/blog/tags/facebook',
    component: ComponentCreator('/book/blog/tags/facebook', '6bd'),
    exact: true
  },
  {
    path: '/book/blog/tags/hello',
    component: ComponentCreator('/book/blog/tags/hello', '22a'),
    exact: true
  },
  {
    path: '/book/blog/tags/hola',
    component: ComponentCreator('/book/blog/tags/hola', '613'),
    exact: true
  },
  {
    path: '/book/blog/welcome',
    component: ComponentCreator('/book/blog/welcome', 'e2e'),
    exact: true
  },
  {
    path: '/book/markdown-page',
    component: ComponentCreator('/book/markdown-page', '751'),
    exact: true
  },
  {
    path: '/book/docs',
    component: ComponentCreator('/book/docs', '3a5'),
    routes: [
      {
        path: '/book/docs',
        component: ComponentCreator('/book/docs', 'f76'),
        routes: [
          {
            path: '/book/docs',
            component: ComponentCreator('/book/docs', 'fda'),
            routes: [
              {
                path: '/book/docs/chapter-1/',
                component: ComponentCreator('/book/docs/chapter-1/', '0d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-1/assessment',
                component: ComponentCreator('/book/docs/chapter-1/assessment', '34f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-1/assignments',
                component: ComponentCreator('/book/docs/chapter-1/assignments', 'b9d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-1/lessons/lesson-1.1/',
                component: ComponentCreator('/book/docs/chapter-1/lessons/lesson-1.1/', 'e63'),
                exact: true
              },
              {
                path: '/book/docs/chapter-1/lessons/lesson-1.2/',
                component: ComponentCreator('/book/docs/chapter-1/lessons/lesson-1.2/', 'fd3'),
                exact: true
              },
              {
                path: '/book/docs/chapter-1/purpose',
                component: ComponentCreator('/book/docs/chapter-1/purpose', '3f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-1/technical-integration',
                component: ComponentCreator('/book/docs/chapter-1/technical-integration', 'eac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-2/',
                component: ComponentCreator('/book/docs/chapter-2/', '0fe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-2/assessment',
                component: ComponentCreator('/book/docs/chapter-2/assessment', '57d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-2/assignments',
                component: ComponentCreator('/book/docs/chapter-2/assignments', '38c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-2/purpose',
                component: ComponentCreator('/book/docs/chapter-2/purpose', '04a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-2/technical-integration',
                component: ComponentCreator('/book/docs/chapter-2/technical-integration', '224'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-3/',
                component: ComponentCreator('/book/docs/chapter-3/', '149'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-3/assessment',
                component: ComponentCreator('/book/docs/chapter-3/assessment', 'd6c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-3/assignments',
                component: ComponentCreator('/book/docs/chapter-3/assignments', '1d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-3/purpose',
                component: ComponentCreator('/book/docs/chapter-3/purpose', 'fd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/chapter-3/technical-integration',
                component: ComponentCreator('/book/docs/chapter-3/technical-integration', 'e34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/intro',
                component: ComponentCreator('/book/docs/intro', '317'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/book/docs/tutorial-basics/congratulations', '432'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/book/docs/tutorial-basics/create-a-blog-post', '1bf'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/book/docs/tutorial-basics/create-a-document', '2a3'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/book/docs/tutorial-basics/create-a-page', '008'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/book/docs/tutorial-basics/deploy-your-site', 'fa6'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/book/docs/tutorial-basics/markdown-features', 'e88'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/book/docs/tutorial-extras/manage-docs-versions', '79d'),
                exact: true
              },
              {
                path: '/book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/book/docs/tutorial-extras/translate-your-site', '45e'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/book/',
    component: ComponentCreator('/book/', 'bfd'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
