import { defineCollection } from 'astro:content';
import { glob } from 'astro/loaders';
import { z } from 'astro/zod';
import { algorithmSchema } from '../schema/algorithm';
import { registeredAlgorithms } from './lib/registered-algorithms';

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
    loader: registeredAlgorithms(),
    // Reject unknown display metadata; retain keywords, curated scenes, compact showcases and video alternatives.
    schema: algorithmSchema.strict().describe('Algorithm display metadata with compact showcases with static or animated cards and method sources'),
  }),
};
