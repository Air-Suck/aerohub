import { defineConfig } from 'astro/config';
import { satteri } from '@astrojs/markdown-satteri';
import responsiveTables from './src/lib/responsive-tables.mjs';

export default defineConfig({
  output: 'static',
  site: process.env.SITE_URL || undefined,
  base: process.env.BASE_PATH || '/',
  trailingSlash: 'always',
  markdown: { processor: satteri({ hastPlugins: [responsiveTables] }) },
});
