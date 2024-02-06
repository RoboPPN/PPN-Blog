import React from 'react'
import { Variants, motion, useScroll, useTransform } from 'framer-motion' // Import motion from framer-motion

import Translate from '@docusaurus/Translate'

import HeroMain from './img/hero_main.svg'

import styles from './styles.module.scss'
import SocialLinks from '@site/src/components/SocialLinks'

import { Icon, IconProps } from '@iconify/react'

const variants: Variants = {
  visible: i => ({
    opacity: 1,
    y: 0,
    transition: {
      type: 'spring',
      damping: 25,
      stiffness: 100,
      duration: 0.3,
      delay: i * 0.3,
    },
  }),
  hidden: { opacity: 0, y: 30 },
}

function Logos() {
  const { scrollYProgress } = useScroll()

  // 往下滚动 元素向上移动
  const y1 = useTransform(scrollYProgress, [0, 1], ['0%', '-500%'], {
    clamp: false,
  })

  // 往下滚动 元素向下移动
  const y2 = useTransform(scrollYProgress, [0, 1], ['0%', '500%'], {
    clamp: false,
  })

  // 图标网址：https://icon-sets.iconify.design/
  const logos: IconProps[] = [

    {
      icon: 'logos:google',
      style: { left: '12%', top: '30%' },
    },
    {
      icon: 'logos:git-icon',
      style: { left: '19%', top: '30%' },
    },
    {
      icon: 'logos:visual-studio-code',
      style: { left: '26%', top: '30%' },
    },
    {
      icon: 'logos:markdown',
      style: { left: '33%', top: '30%' },
    },
    {
      icon: 'logos:docusaurus',
      style: { left: '40%', top: '30%' },
    },


    {
      icon: 'logos:github',
      style: { left: '12%', top: '57%' },
    },
    {
      icon: 'devicon:cplusplus',
      style: { top: '47%', left: '38%' },
    },
    {
      icon: 'devicon:python',
      style: { top: '47%', left: '43%' },
    },
    {
      icon: 'logos:ros',
      style: { top: '54%', left: '38%'  },
    },
    {
      icon: 'logos:opencv',
      style: { top: '54%', left: '43%' },
    },
    {
      icon: 'logos:mysql',
      style: { top: '61%', left: '38%' },
    },
    {
      icon: 'logos:arm',
      style: { top: '61%', left: '43%' },
    },

    {
      icon: 'logos:ubuntu',
      style: { bottom: '13%', right: '13%' },
    },
  ]

  return (
    <>
      {logos.map((l, index) => {
        const yValue = index % 2 === 0 ? y1 : y2

        return (
          <motion.div
            className={styles.box}
            initial={{ opacity: 0.01, y: 50 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{
              duration: Math.random() * 2 + 0.5,
              delay: 0.5,
            }}
            style={{
              ...l.style,
              y: yValue,
            }}
          >
            <Icon icon={l.icon}></Icon>
          </motion.div>
        )
      })}
    </>
  )
}

function Background() {
  return (
    <>
      <motion.div className={styles.background}>
        <Logos />
        <HeroMain />
        <div className={styles.circle} />
      </motion.div>
    </>
  )
}

function Name() {
  return (
    <motion.div
      className={styles.hero_text}
      custom={1}
      initial="hidden"
      animate="visible"
      variants={variants}
      onMouseMove={e => {
        e.currentTarget.style.setProperty('--x', `${e.clientX}px`)
        e.currentTarget.style.setProperty('--y', `${e.clientY}px`)
      }}
    >
      <Translate id="homepage.hero.greet">Hello👋! 我是帕帕尼</Translate>
       {/* <span className={styles.wave}>👋</span> */}
    </motion.div>
  )
}

export default function Hero() {
  return (
    <motion.div className={styles.hero}>
      <div className={styles.intro}>
        <Name />
        <motion.p
          custom={2}
          initial="hidden"
          animate="visible"
          variants={variants}
        >
          <Translate id="homepage.hero.text">
            {`我在这里记录一些知识，以免遗忘，也希望对你有用，请随意浏览～`}
          </Translate>
        </motion.p>

        {/* 社交链接 */}
        <motion.div
          custom={3}
          initial="hidden"
          animate="visible"
          variants={variants}
        >
          <SocialLinks />
        </motion.div>
        
        {/* 自我介绍 */}
        <motion.div
          className={styles.buttonGroup}
          custom={4}
          initial="hidden"
          animate="visible"
          variants={variants}
        >
          <div className={styles.outer}>
            <div className={styles.gradient} />
            <a className={styles.button} href={'./about'}>
              <Translate id="hompage.hero.introduce">自我介绍</Translate>
            </a>
          </div>
        </motion.div>
      </div>
      <Background />
    </motion.div>
  )
}
