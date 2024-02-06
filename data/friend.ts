export const Friends: Friend[] = [
  {
    title: '峰华前端工程师',
    description: '致力于帮助你以最直观、最快速的方式学会前端开发',
    website: 'https://zxuqian.cn',
    avatar: '/img/friend/zxuqian.png',
  },
  {
    title: '愧怍',
    description: ' 保持学习，希望在有限的时间内，学到无限的可能',
    website: 'https://kuizuo.cn/',
    avatar: '/img/friend/kuizuo.png',
  },
  {
    title: "Power's Wiki",
    description: ' 博览万物，融会贯通 ',
    website: 'https://wiki-power.com/',
    avatar: '/img/friend/power-lin.png',
  },
  {
    title: "Shake's Blog",
    description: ' 这里是加减的 Blog 😉 ',
    website: 'https://www.shaking.site/',
    avatar: '/img/friend/shake.png',
  },
  {
    title: "尚宇",
    description: ' 星海横流，岁月成碑 ',
    website: 'https://www.disnox.top/',
    avatar: '/img/friend/shangyu.png',
  },
];

export type Friend = {
  title: string
  description: string
  website: string
  avatar?: any
}
