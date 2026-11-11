import { defineCollection } from 'astro:content';
import { glob } from 'astro/loaders';
import { z } from 'astro/zod';
import { statSync } from 'node:fs';
import registry from '../content/gallery.json';
import { algorithmSchema } from '../schema/algorithm';

const { items } = z.object({
  items: z.array(z.object({
    id: z.string().regex(/^[a-z0-9]+(?:[_-][a-z0-9]+)*$/),
    markdown: z.string().regex(/^content\/[a-zA-Z0-9_/-]+\.md$/,
      'Use a Markdown path inside gallery/content/'),
  }).strict()),
}).strict().parse(registry);

const ids = new Set<string>();
const paths = new Map<string, string>();
for (const item of items) {
  if (ids.has(item.id) || paths.has(item.markdown)) {
    throw new Error(`Duplicate Gallery ID or Markdown path: ${item.id}`);
  }
  if (!statSync(new URL(`../${item.markdown}`, import.meta.url)).isFile()) {
    throw new Error(`Missing Gallery Markdown: ${item.markdown}`);
  }
  ids.add(item.id);
  paths.set(item.markdown, item.id);
}

export const collections = {
  docs: defineCollection({
    loader: glob({ base: './content/docs', pattern: '**/*.md' }),
    schema: z.object({
      title: z.string().min(1),
      description: z.string().min(1),
      section: z.enum(['开始使用', '用户指南', '深入了解']),
      order: z.number(),
      status: z.enum(['初版', '待完善']),
    }).strict(),
  }),
  homepage: defineCollection({
    loader: glob({ base: './src/content', pattern: 'homepage.md' }),
    schema: z.object({
      title: z.string().min(1),
      eyebrow: z.string().min(1),
      heroTitle: z.string().min(1),
      lead: z.string().min(1),
      features: z.array(z.object({ title: z.string().min(1), description: z.string().min(1) })),
      galleryLinkLabel: z.string().min(1),
    }),
  }),
  algorithms: defineCollection({
    loader: items.length === 0 ? async () => [] : glob({
      base: './content/',
      pattern: items.map((item) => item.markdown.slice('content/'.length)),
      generateId: ({ entry }) => {
        const id = paths.get(`content/${entry}`);
        if (!id) throw new Error(`Unregistered Gallery Markdown: ${entry}`);
        return id;
      },
    }),
    // Reject unknown metadata instead of silently dropping fields from the page.
    schema: algorithmSchema.strict(),
  }),
};
