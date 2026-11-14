import { readFileSync, statSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { glob } from 'astro/loaders';
import type { Loader, LoaderContext } from 'astro/loaders';
import { z } from 'astro/zod';

const registrySchema = z.object({
  items: z.array(z.object({
    id: z.string().regex(/^[a-z0-9]+(?:[_-][a-z0-9]+)*$/),
    markdown: z.string().regex(/^content\/[a-zA-Z0-9_/-]+\.md$/,
      'Use a Markdown path inside gallery/content/'),
  }).strict()),
}).strict();

/** Read the registry on every refresh, not once when content.config.ts is evaluated. */
export function registeredAlgorithms(): Loader {
  return {
    name: 'registered-algorithms',
    async load(context: LoaderContext) {
      const registryURL = new URL('content/gallery.json', context.config.root);
      const registryPath = fileURLToPath(registryURL);
      const watcher = context.watcher;
      const listeners: Array<{ event: string; callback: (...args: any[]) => void }> = [];
      // Replacing the glob must also replace its listeners, without touching other collections.
      const scopedWatcher = watcher && new Proxy(watcher, {
        get(target, prop) {
          if (prop === 'on') return (event: string, callback: (...args: any[]) => void) => {
            listeners.push({ event, callback });
            target.on(event, callback);
            return scopedWatcher;
          };
          const value = Reflect.get(target, prop);
          return typeof value === 'function' ? value.bind(target) : value;
        },
      });
      async function refresh() {
        const { items } = registrySchema.parse(JSON.parse(readFileSync(registryURL, 'utf8')));
        const ids = new Set<string>();
        const paths = new Map<string, string>();
        for (const item of items) {
          if (ids.has(item.id) || paths.has(item.markdown)) {
            throw new Error(`Duplicate Gallery ID or Markdown path: ${item.id}`);
          }
          if (!statSync(new URL(item.markdown, context.config.root)).isFile()) {
            throw new Error(`Missing Gallery Markdown: ${item.markdown}`);
          }
          ids.add(item.id);
          paths.set(item.markdown, item.id);
        }
        for (const { event, callback } of listeners.splice(0)) watcher?.off(event, callback);
        if (items.length === 0) {
          context.store.clear();
          return;
        }
        await glob({
          base: './content/',
          pattern: items.map((item) => item.markdown.slice('content/'.length)),
          generateId: ({ entry }) => {
            const id = paths.get(`content/${entry}`);
            if (!id) throw new Error(`Unregistered Gallery Markdown: ${entry}`);
            return id;
          },
        }).load({ ...context, watcher: scopedWatcher });
      }
      await refresh();
      if (watcher) {
        watcher.add(registryPath);
        let pending = Promise.resolve();
        const onRegistryChange = (path: string) => {
          if (path !== registryPath) return;
          pending = pending.then(refresh).then(() => {
            context.logger.info('Reloaded Gallery registry');
          }).catch((error) => context.logger.error(`Failed to reload Gallery registry: ${error.message}`));
        };
        watcher.on('change', onRegistryChange);
        watcher.on('add', onRegistryChange);
      }
    },
  };
}
