const path = require('path')
const beian = 'CC BY-NC 4.0'

module.exports = async function createConfigAsync() {
  return {
    title: '帕帕尼的小站',
    url: 'https://roboppn.top',
    baseUrl: '/',
    favicon: 'img/PPN.png',
    organizationName: 'RoboPPN',
    projectName: 'blog',
    tagline: '积跬步成千里，积小流成江海',
    themeConfig: {
      image: 'img/logo.png',
      docs: {
        sidebar: {
          hideable: true,
        },
      },
      navbar: {
        logo: {
          alt: '帕帕尼',
          src: 'img/PPN.png',
          srcDark: 'img/PPN.png',
        },
        hideOnScroll: true,
        items: [
          // {
          //   label: '📝学习',
          //   position: 'right',
          //   items: [
          //     {
          //       label: '📑技术笔记',
          //       to: 'docs/note-introduction/',
          //     },
          //     {
          //       label: '🗂️项目介绍',
          //       to: 'docs/project_group/',
          //     },
          //     {
          //       label: '📂高效工作指南',
          //       to: 'docs/tools/',
          //     },
          //   ],
          // },
          {
            label: '📑笔记',
            position: 'right',
            to: 'docs/note-introduction/',
          },
          {
            label: '📘博客',
            position: 'right',
            to: 'blog',
          },
          {
            label: '🤖项目',
            position: 'right',
            to: 'project',
          },
          {
            label: '更多',
            position: 'right',
            items: [
              {
                label: '归档',
                to: 'blog/archive',
              },
              {
                label: '资源',
                to: 'resource',
              },
              {
                label:  '友链',
                to: 'friends',
              },
              // {
              //   label: '工具推荐',
              //   to: 'docs/tools/',
              // },
            ],
          },
          {
            href: 'https://github.com/RoboPPN/PPN-Blog', className: 'header-github-link',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `本站所有内容遵循<a href="https://creativecommons.org/licenses/by/4.0/deed.zh" >${beian}</a>开源协议，仅限非商业性使用，转载请注明出处</p><p>Copyright © 2022 - PRESENT 帕帕尼 Built with Docusaurus.</p>`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/vsLight'),
        darkTheme: require('prism-react-renderer/themes/vsDark'),
        additionalLanguages: ['java', 'php', 'rust', 'toml'],
        defaultLanguage: 'javascript',
        magicComments: [
          {
            className: 'theme-code-block-highlighted-line',
            line: 'highlight-next-line',
            block: { start: 'highlight-start', end: 'highlight-end' },
          },
          {
            className: 'code-block-error-line',
            line: 'This will error',
          },
        ],
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
      algolia: {
        appId: 'KB96Z4PFJM',
        apiKey: '00e9d782a3f2d712b1fe0d7907716ac7',
        indexName: 'ppn_index',
      },
      zoom: {
        selector: '.markdown :not(em) > img',
        background: {
          light: 'rgb(255, 255, 255)',
          dark: 'rgb(50, 50, 50)',
        },
        config: {},
      },
      giscus: {
        repo: 'disnox/disnox_blog',
        repoId: 'R_kgDOIS7BTg',
        category: 'General',
        categoryId: 'DIC_kwDOIS7BTs4CSN3O',
        theme: 'light',
        darkTheme: 'dark',
      },
      liveCodeBlock: {
        playgroundPosition: 'top',
      },
      socials: {
        github: 'https://github.com/disnox',
        blibli: 'https://space.bilibili.com/511798206?spm_id_from=333.1007.0.0',
        zhihu: 'https://www.zhihu.com/people/chui-zi-26-38',
        csdn: 'https://blog.csdn.net/m0_47339333?spm=1000.2115.3001.5343',
        qq: 'https://www.helloimg.com/image/ZV1U6u',
        wx: 'http://n0i.cn/1DmzeU',
        cloudmusic: 'https://music.163.com/#/user/home?id=3906202648',
      },
    },
    headTags: [
      {
        tagName: 'meta',
        attributes: {
          name: 'description',
          content: '帕帕尼的个人博客',
        },
      },
    ],
    presets: [
      [
        '@docusaurus/preset-classic',
        /** @type {import('@docusaurus/preset-classic').Options} */
        ({
          docs: {
            path: 'docs',
            sidebarPath: 'sidebars.js',
            remarkPlugins: [(await import('remark-math')).default],
            rehypePlugins: [(await import('rehype-katex')).default],
          },
          blog: false,
          theme: {
            customCss: [require.resolve('./src/css/custom.scss')],
          },
          sitemap: {
            changefreq: 'daily',
            priority: 0.5,
          },
          gtag: {
            trackingID: 'G-S4SD5NXWXF',
            anonymizeIP: true,
          },
          // debug: true,
        }),
      ],
    ],
    plugins: [
      'docusaurus-plugin-image-zoom',
      'docusaurus-plugin-sass',
      path.resolve(__dirname, './src/plugin/plugin-baidu-tongji'),
      path.resolve(__dirname, './src/plugin/plugin-baidu-push'),
      [
        path.resolve(__dirname, './src/plugin/plugin-content-blog'),
        {
          path: 'blog',
          editUrl: ({ locale, blogDirPath, blogPath, permalink }) =>
            `https://github.com/disnox/blog/edit/main/${blogDirPath}/${blogPath}`,
          editLocalizedFiles: false,
          blogDescription: '帕帕尼的个人博客',
          blogSidebarCount: 10,
          blogSidebarTitle: 'Blogs',
          postsPerPage: 10,
          showReadingTime: true,
          readingTime: ({ content, frontMatter, defaultReadingTime }) =>
            defaultReadingTime({ content, options: { wordsPerMinute: 300 } }),
          feedOptions: {
            type: 'all',
            title: '帕帕尼',
            copyright: `Copyright © ${new Date().getFullYear()} 帕帕尼 Built with Docusaurus.<p><a href="http://beian.miit.gov.cn/" class="footer_lin">${beian}</a></p>`,
          },
        },
      ],
      [
        '@docusaurus/plugin-ideal-image',
        {
          disableInDev: false,
        },
      ],
      [
        '@docusaurus/plugin-pwa',
        {
          debug: true,
          offlineModeActivationStrategies: [
            'appInstalled',
            'standalone',
            'queryString',
          ],
          pwaHead: [
            {
              tagName: 'link',
              rel: 'icon',
              href: '/img/logo.png',
            },
            {
              tagName: 'link',
              rel: 'manifest',
              href: '/manifest.json',
            },
            {
              tagName: 'meta',
              name: 'theme-color',
              content: 'rgb(51 139 255)',
            },
          ],
        },
      ],
    ],
    stylesheets: [
      {
        href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
        type: 'text/css',
        integrity:
          'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
        crossorigin: 'anonymous',
      },
    ],
    i18n: {
      defaultLocale: 'zh-CN',
      locales: ['en', 'zh-CN'],
      localeConfigs: {
        en: {
          htmlLang: 'en-GB',
        },
      },
    },
  }
}