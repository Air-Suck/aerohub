import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { dev } from 'astro';
import registry from '../content/gallery.json' with { type: 'json' };

const root = new URL('../', import.meta.url);
const repository = fileURLToPath(new URL('../', root));
const violations = [];
const server = await dev({
  root,
  server: { host: '127.0.0.1', port: 0 },
  vite: {
    plugins: [{
      name: 'check-gallery-watch-scope',
      configureServer(vite) {
        const add = vite.watcher.add.bind(vite.watcher);
        vite.watcher.add = (paths) => {
          const list = Array.isArray(paths) ? paths : [paths];
          // Block the regression before it recursively consumes system watchers.
          const allowed = list.filter((path) => {
            if (resolve(path) !== resolve(repository)) return true;
            violations.push(path);
            return false;
          });
          return add(allowed);
        };
      },
    }],
  },
});

try {
  assert.deepEqual(violations, [], 'Content loader must not watch the repository root');
  for (const path of ['/', '/gallery/', ...registry.items.map((item) => `/gallery/${item.id}/`)]) {
    const response = await fetch(`http://127.0.0.1:${server.address.port}${path}`);
    assert.equal(response.status, 200, path);
  }
  const directories = Object.keys(server.watcher.getWatched());
  assert(!directories.some((path) => path.startsWith(resolve(repository, 'artifacts'))));
  console.log('PASS: Gallery dev pages respond; repository root and artifacts are not watched.');
} finally {
  await server.stop();
}
