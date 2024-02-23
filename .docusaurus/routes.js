import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '3ad'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'c28'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'c2e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'd6e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '9eb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '05c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '721'),
    exact: true
  },
  {
    path: '/about',
    component: ComponentCreator('/about', '473'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '555'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', 'b0e'),
    exact: true
  },
  {
    path: '/blog/blog_write',
    component: ComponentCreator('/blog/blog_write', 'd97'),
    exact: true
  },
  {
    path: '/blog/classic',
    component: ComponentCreator('/blog/classic', '034'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', 'b66'),
    exact: true
  },
  {
    path: '/blog/tags/随笔',
    component: ComponentCreator('/blog/tags/随笔', '9bd'),
    exact: true
  },
  {
    path: '/blog/tags/blog',
    component: ComponentCreator('/blog/tags/blog', '64a'),
    exact: true
  },
  {
    path: '/blog/tags/c-c',
    component: ComponentCreator('/blog/tags/c-c', '4ae'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '57f'),
    exact: true
  },
  {
    path: '/blog/tags/exam',
    component: ComponentCreator('/blog/tags/exam', '448'),
    exact: true
  },
  {
    path: '/blog/tags/linux',
    component: ComponentCreator('/blog/tags/linux', '3ea'),
    exact: true
  },
  {
    path: '/blog/thread_pool1',
    component: ComponentCreator('/blog/thread_pool1', '2f0'),
    exact: true
  },
  {
    path: '/blog/thread_pool2',
    component: ComponentCreator('/blog/thread_pool2', '6e5'),
    exact: true
  },
  {
    path: '/friends/',
    component: ComponentCreator('/friends/', '1d5'),
    exact: true
  },
  {
    path: '/project/',
    component: ComponentCreator('/project/', 'a26'),
    exact: true
  },
  {
    path: '/resource/',
    component: ComponentCreator('/resource/', 'c6e'),
    exact: true
  },
  {
    path: '/search',
    component: ComponentCreator('/search', '2cf'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '552'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '6b4'),
        routes: [
          {
            path: '/docs/tags',
            component: ComponentCreator('/docs/tags', '0cc'),
            exact: true
          },
          {
            path: '/docs/tags/电机',
            component: ComponentCreator('/docs/tags/电机', '835'),
            exact: true
          },
          {
            path: '/docs/tags/工具',
            component: ComponentCreator('/docs/tags/工具', 'fe9'),
            exact: true
          },
          {
            path: '/docs/tags/开发工具',
            component: ComponentCreator('/docs/tags/开发工具', 'c1e'),
            exact: true
          },
          {
            path: '/docs/tags/美化',
            component: ComponentCreator('/docs/tags/美化', '093'),
            exact: true
          },
          {
            path: '/docs/tags/配置',
            component: ComponentCreator('/docs/tags/配置', 'd2c'),
            exact: true
          },
          {
            path: '/docs/tags/数据结构',
            component: ComponentCreator('/docs/tags/数据结构', '256'),
            exact: true
          },
          {
            path: '/docs/tags/硬件',
            component: ComponentCreator('/docs/tags/硬件', '7e0'),
            exact: true
          },
          {
            path: '/docs/tags/c-c',
            component: ComponentCreator('/docs/tags/c-c', 'd09'),
            exact: true
          },
          {
            path: '/docs/tags/foc',
            component: ComponentCreator('/docs/tags/foc', '4c4'),
            exact: true
          },
          {
            path: '/docs/tags/linux',
            component: ComponentCreator('/docs/tags/linux', '083'),
            exact: true
          },
          {
            path: '/docs/tags/pmsm',
            component: ComponentCreator('/docs/tags/pmsm', 'b55'),
            exact: true
          },
          {
            path: '/docs/tags/st',
            component: ComponentCreator('/docs/tags/st', '676'),
            exact: true
          },
          {
            path: '/docs/tags/system',
            component: ComponentCreator('/docs/tags/system', '342'),
            exact: true
          },
          {
            path: '/docs/tags/terminal',
            component: ComponentCreator('/docs/tags/terminal', 'ea9'),
            exact: true
          },
          {
            path: '/docs/tags/vscode',
            component: ComponentCreator('/docs/tags/vscode', '929'),
            exact: true
          },
          {
            path: '/docs',
            component: ComponentCreator('/docs', '622'),
            routes: [
              {
                path: '/docs/AxDrive-L_hardware_design_report',
                component: ComponentCreator('/docs/AxDrive-L_hardware_design_report', 'da3'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/AxDrive-L_software_design_report',
                component: ComponentCreator('/docs/AxDrive-L_software_design_report', '11b'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/AxDrive-L_user_manual',
                component: ComponentCreator('/docs/AxDrive-L_user_manual', 'e0e'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/binary_tree',
                component: ComponentCreator('/docs/binary_tree', 'f77'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_array',
                component: ComponentCreator('/docs/C_array', '5d5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_basic_syntax',
                component: ComponentCreator('/docs/C_basic_syntax', 'f62'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_brief_introduction',
                component: ComponentCreator('/docs/C_brief_introduction', '846'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_character_string',
                component: ComponentCreator('/docs/C_character_string', 'e73'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_control_flow',
                component: ComponentCreator('/docs/C_control_flow', '805'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_datetype',
                component: ComponentCreator('/docs/C_datetype', '75f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_environment_settings',
                component: ComponentCreator('/docs/C_environment_settings', '809'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_function',
                component: ComponentCreator('/docs/C_function', '7f9'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_function_order',
                component: ComponentCreator('/docs/C_function_order', '9c4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_keyword',
                component: ComponentCreator('/docs/C_keyword', 'e92'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_memory',
                component: ComponentCreator('/docs/C_memory', '6e5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_operator',
                component: ComponentCreator('/docs/C_operator', '16c'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_other',
                component: ComponentCreator('/docs/C_other', '2fe'),
                exact: true
              },
              {
                path: '/docs/C_pointer',
                component: ComponentCreator('/docs/C_pointer', '4fe'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_pointer_and_array',
                component: ComponentCreator('/docs/C_pointer_and_array', '2a2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_pointer_array_and_pointer_array',
                component: ComponentCreator('/docs/C_pointer_array_and_pointer_array', '386'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_pointer_High_level_issues',
                component: ComponentCreator('/docs/C_pointer_High_level_issues', 'abd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_pretreatment',
                component: ComponentCreator('/docs/C_pretreatment', '2ad'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_program_structure',
                component: ComponentCreator('/docs/C_program_structure', '3f4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_scope',
                component: ComponentCreator('/docs/C_scope', '0e6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_struct',
                component: ComponentCreator('/docs/C_struct', '883'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_struct_size',
                component: ComponentCreator('/docs/C_struct_size', 'c4a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_two_pointer',
                component: ComponentCreator('/docs/C_two_pointer', 'ac1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_typedef',
                component: ComponentCreator('/docs/C_typedef', '663'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_union_enum',
                component: ComponentCreator('/docs/C_union_enum', '727'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/C_variable_constant',
                component: ComponentCreator('/docs/C_variable_constant', '73a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/Cap',
                component: ComponentCreator('/docs/Cap', 'aa1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/电机控制',
                component: ComponentCreator('/docs/category/电机控制', '3d7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/电机控制-1',
                component: ComponentCreator('/docs/category/电机控制-1', 'dd3'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/category/数据结构',
                component: ComponentCreator('/docs/category/数据结构', '094'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/硬件基础',
                component: ComponentCreator('/docs/category/硬件基础', '81b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/元器件基础',
                component: ComponentCreator('/docs/category/元器件基础', 'a37'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/杂项',
                component: ComponentCreator('/docs/category/杂项', '46b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/axdrive-l',
                component: ComponentCreator('/docs/category/axdrive-l', 'be3'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/category/c-复杂类型',
                component: ComponentCreator('/docs/category/c-复杂类型', '3a0'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/c-语言',
                component: ComponentCreator('/docs/category/c-语言', 'ff2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/c-指针',
                component: ComponentCreator('/docs/category/c-指针', 'c11'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/io-编程技术',
                component: ComponentCreator('/docs/category/io-编程技术', 'ce5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/linux',
                component: ComponentCreator('/docs/category/linux', '852'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/linux-网络编程',
                component: ComponentCreator('/docs/category/linux-网络编程', '704'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/linux-系统编程',
                component: ComponentCreator('/docs/category/linux-系统编程', 'e82'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/cond',
                component: ComponentCreator('/docs/cond', '486'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/data_structure',
                component: ComponentCreator('/docs/data_structure', '1cf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/dc-dc',
                component: ComponentCreator('/docs/dc-dc', '77f'),
                exact: true
              },
              {
                path: '/docs/diary',
                component: ComponentCreator('/docs/diary', 'f3e'),
                exact: true
              },
              {
                path: '/docs/dir',
                component: ComponentCreator('/docs/dir', '9af'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/double_linked_list',
                component: ComponentCreator('/docs/double_linked_list', '0f3'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/everything-quick-search-local-files',
                component: ComponentCreator('/docs/everything-quick-search-local-files', 'd41'),
                exact: true,
                sidebar: "tools"
              },
              {
                path: '/docs/file',
                component: ComponentCreator('/docs/file', '097'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/inline_list',
                component: ComponentCreator('/docs/inline_list', 'cfd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/ipc',
                component: ComponentCreator('/docs/ipc', '7c2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/linear_table',
                component: ComponentCreator('/docs/linear_table', 'dd2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/Linux_introduction',
                component: ComponentCreator('/docs/Linux_introduction', '303'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/Linux_shell',
                component: ComponentCreator('/docs/Linux_shell', 'afb'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/look-up-port-and-kill-process',
                component: ComponentCreator('/docs/look-up-port-and-kill-process', '246'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/MOS',
                component: ComponentCreator('/docs/MOS', 'ebd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/msg',
                component: ComponentCreator('/docs/msg', '2ad'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/mutex',
                component: ComponentCreator('/docs/mutex', '6f7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/pipe',
                component: ComponentCreator('/docs/pipe', '0ac'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/pmsm_motor',
                component: ComponentCreator('/docs/pmsm_motor', '9dd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/posix',
                component: ComponentCreator('/docs/posix', '611'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/pro_api',
                component: ComponentCreator('/docs/pro_api', '285'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/process',
                component: ComponentCreator('/docs/process', '412'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/project_group',
                component: ComponentCreator('/docs/project_group', 'fb7'),
                exact: true,
                sidebar: "project_group"
              },
              {
                path: '/docs/queue',
                component: ComponentCreator('/docs/queue', '796'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/reen',
                component: ComponentCreator('/docs/reen', '02a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/Res',
                component: ComponentCreator('/docs/Res', '736'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/rwlock',
                component: ComponentCreator('/docs/rwlock', '9c1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/sem',
                component: ComponentCreator('/docs/sem', '4b8'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/shm',
                component: ComponentCreator('/docs/shm', 'a56'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/signal',
                component: ComponentCreator('/docs/signal', 'ea6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/single_linked_list',
                component: ComponentCreator('/docs/single_linked_list', '054'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/singly_loop_linked_list',
                component: ComponentCreator('/docs/singly_loop_linked_list', '60f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill',
                component: ComponentCreator('/docs/skill', '69a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/socket',
                component: ComponentCreator('/docs/socket', 'd04'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/stack',
                component: ComponentCreator('/docs/stack', 'a68'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/standard_io',
                component: ComponentCreator('/docs/standard_io', 'a29'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/stat',
                component: ComponentCreator('/docs/stat', '6cf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/system_io',
                component: ComponentCreator('/docs/system_io', 'c60'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/tcp',
                component: ComponentCreator('/docs/tcp', '9ef'),
                exact: true
              },
              {
                path: '/docs/thread',
                component: ComponentCreator('/docs/thread', '054'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/thread_pool',
                component: ComponentCreator('/docs/thread_pool', '8ac'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/thread_scheduling',
                component: ComponentCreator('/docs/thread_scheduling', '55b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/tools',
                component: ComponentCreator('/docs/tools', '47a'),
                exact: true,
                sidebar: "tools"
              },
              {
                path: '/docs/vscode-config',
                component: ComponentCreator('/docs/vscode-config', '402'),
                exact: true,
                sidebar: "tools"
              },
              {
                path: '/docs/windows-custom-right-click-menu',
                component: ComponentCreator('/docs/windows-custom-right-click-menu', 'fd0'),
                exact: true,
                sidebar: "tools"
              },
              {
                path: '/docs/windows-terminal-beautify',
                component: ComponentCreator('/docs/windows-terminal-beautify', 'df7'),
                exact: true,
                sidebar: "tools"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '99f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
