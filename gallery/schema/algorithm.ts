import { z } from 'astro/zod';

export const algorithmSchema = z.object({
  title: z.string().min(1),
  summary: z.string().min(1),
  category: z.enum(['planner', 'behavior', 'state_estimator', 'perception']),
  source: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) source URL'),
  revision: z.string().min(1).optional(),
  integration: z.enum(['not_started', 'in_progress', 'integrated']),
  validation: z.enum(['unverified', 'build', 'interface', 'simulation', 'hardware']),
  evidence: z.array(z.object({
    label: z.string().min(1),
    url: z.url().refine((url) => /^https?:\/\//.test(url), 'Use an HTTP(S) evidence URL'),
  })).default([]),
}).superRefine((data, ctx) => {
  if (data.validation !== 'unverified' && (!data.revision || data.evidence.length === 0)) {
    ctx.addIssue({ code: 'custom', message: '验证结论必须关联源码版本和公开证据；尚未整理时使用 unverified。' });
  }
});
