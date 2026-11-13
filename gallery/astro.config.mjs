import { defineConfig } from 'astro/config';
import { satteri } from '@astrojs/markdown-satteri';
import responsiveTables from './src/lib/responsive-tables.mjs';

export default defineConfig({
  output: 'static',
  image: { endpoint: { route: '/_image', entrypoint: './src/lib/image-endpoint.ts' } },
  // Keep content caches local even when node_modules is shared with another worktree.
  cacheDir: './.astro/cache/',
  site: process.env.SITE_URL || undefined,
  base: process.env.BASE_PATH || '/',
  trailingSlash: 'always',
  markdown: { processor: satteri({ hastPlugins: [responsiveTables] }) },
});
