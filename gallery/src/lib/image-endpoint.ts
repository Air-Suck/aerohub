import type { APIRoute } from 'astro';

// Keep Astro's image handling, but avoid stale same-size images during editing.
export const GET: APIRoute = async (context) => {
  const endpoint = import.meta.env.DEV
    ? await import('astro/assets/endpoint/dev')
    : await import('astro/assets/endpoint/generic');
  const response = await endpoint.GET(context);
  if (import.meta.env.DEV) response.headers.set('Cache-Control', 'no-store');
  return response;
};
