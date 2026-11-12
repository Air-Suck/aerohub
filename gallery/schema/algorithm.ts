import { z } from 'astro/zod';

export const algorithmSchema = z.object({
  title: z.string().min(1),
  summary: z.string().min(1),
  keywords: z.array(z.string().min(1)).max(8).default([]),
  category: z.enum(['planner', 'behavior', 'state_estimator', 'perception']),
  source: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) source URL'),
  revision: z.string().min(1).optional(),
  demo: z.object({
    image: z.union([
      z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) image URL'),
      z.string().regex(/^content\/algorithms\/[a-z0-9_-]+\/assets\/[a-zA-Z0-9_.-]+\.(?:jpg|jpeg|png|webp)$/),
    ]),
    alt: z.string().min(1),
    caption: z.string().min(1),
    source: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) source URL'),
    video: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) video URL'),
  }).optional(),
  showcase: z.object({
    headline: z.string().min(1),
    methodSources: z.array(z.object({ label: z.string().min(1), url: z.url().refine(url => /^https?:\/\//.test(url), 'Use an HTTP(S) source URL') })).optional(),
    sourceNote: z.string().min(1).optional(),
    highlights: z.array(z.object({ title: z.string().min(1), text: z.string().min(1) })).min(1).max(3),
  }).optional(),
  media: z.array(z.object({
    title: z.string().min(1),
    image: z.string().regex(/^content\/algorithms\/[a-z0-9_-]+\/assets\/[a-zA-Z0-9_.-]+\.webp$/),
    animation: z.string().regex(/^content\/algorithms\/[a-z0-9_-]+\/assets\/[a-zA-Z0-9_.-]+\.webp$/),
    alt: z.string().min(1),
    caption: z.string().min(1),
    source: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) source URL'),
  })).max(6).default([]),
  videos: z.array(z.object({
    label: z.string().min(1),
    url: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) video URL'),
  })).max(6).default([]),
  integration: z.enum(['not_started', 'in_progress', 'integrated']),
  validation: z.enum(['unverified', 'build', 'interface', 'simulation', 'hardware']),
  evidence: z.array(z.object({
    label: z.string().min(1),
    url: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) evidence URL'),
  })).default([]),
}).superRefine((data, ctx) => {
  if (data.showcase && !data.demo) {
    ctx.addIssue({ code: 'custom', message: '紧凑展示需要配置主视频 demo。' });
  }
  if (data.validation !== 'unverified' && (!data.revision || data.evidence.length === 0)) {
    ctx.addIssue({ code: 'custom', message: '验证结论必须关联源码版本和公开证据；尚未整理时使用 unverified。' });
  }
});
