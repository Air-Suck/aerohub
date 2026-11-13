import assert from 'node:assert/strict';
import { cpSync, mkdirSync, mkdtempSync, readFileSync, rmSync, symlinkSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { join } from 'node:path';
import { dev } from 'astro';

const root = fileURLToPath(new URL('../', import.meta.url));
mkdirSync(join(root, '.astro'), { recursive: true });
const fixture = mkdtempSync(join(root, '.astro/registry-watch-'));
let server;
try {
  for (const path of ['src', 'schema', 'content', 'astro.config.mjs', 'package.json']) {
    cpSync(join(root, path), join(fixture, path), { recursive: true });
  }
  symlinkSync(join(root, 'node_modules'), join(fixture, 'node_modules'), 'dir');
  // Optional diagnostic mode reproduces the previous module-evaluation snapshot.
  if (process.argv.includes('--legacy')) {
    const path = join(fixture, 'src/content.config.ts');
    let source = readFileSync(path, 'utf8');
    source = source.replace("import { registeredAlgorithms } from './lib/registered-algorithms';",
      "import registry from '../content/gallery.json';\nconst snapshot = registry.items;");
    source = source.replace('loader: registeredAlgorithms(),', `loader: glob({ base: './content/', pattern: snapshot.map(x => x.markdown.slice('content/'.length)), generateId: ({entry}) => snapshot.find(x => x.markdown === 'content/' + entry).id }),`);
    writeFileSync(path, source);
  }
  server = await dev({ root: fixture, server: { host: '127.0.0.1', port: 0 } });
  const base = `http://127.0.0.1:${server.address.port}`;
  const registryPath = join(fixture, 'content/gallery.json');
  const registry = JSON.parse(readFileSync(registryPath, 'utf8'));
  const probe = { id: 'registry_watch_probe', markdown: 'content/algorithms/registry_watch_probe/index.md' };
  const probeDir = join(fixture, 'content/algorithms/registry_watch_probe');
  mkdirSync(probeDir, { recursive: true });
  writeFileSync(join(probeDir, 'index.md'), `---\ntitle: Registry Watch Probe\nsummary: Isolated development registry test.\ncategory: planner\nsource: https://github.com/hku-mars/IPC\nintegration: not_started\nvalidation: unverified\n---\n\nProbe content.\n`);
  async function expectVisibility(visible, label) {
    const deadline = Date.now() + 10000;
    let observed;
    do {
      const list = await fetch(`${base}/gallery/`);
      const listHTML = await list.text();
      const detail = await fetch(`${base}/gallery/${probe.id}/`);
      const detailHTML = await detail.text();
      observed = { list: list.status, listed: listHTML.includes(`/gallery/${probe.id}/`), detail: detail.status, rendered: detailHTML.includes('Registry Watch Probe') };
      if (observed.list === 200 && observed.listed === visible && observed.detail === (visible ? 200 : 404) && (!visible || observed.rendered)) {
        console.log(`PASS: ${label}`);
        return;
      }
      await new Promise(resolve => setTimeout(resolve, 200));
    } while (Date.now() < deadline);
    assert.fail(`${label}: ${JSON.stringify(observed)}`);
  }
  await expectVisibility(false, 'unregistered Markdown excluded');
  writeFileSync(registryPath, JSON.stringify({ items: [...registry.items, probe] }, null, 2));
  await expectVisibility(true, 'new registry item appears without restarting server');
  writeFileSync(registryPath, JSON.stringify(registry, null, 2));
  await expectVisibility(false, 'removed registry item disappears without restarting server');
  writeFileSync(registryPath, JSON.stringify({ items: [] }));
  const deadline = Date.now() + 10000;
  let empty = false;
  do {
    const response = await fetch(`${base}/gallery/`);
    empty = response.status === 200 && (await response.text()).includes('尚无算法条目');
    if (!empty) await new Promise(resolve => setTimeout(resolve, 200));
  } while (!empty && Date.now() < deadline);
  assert(empty, 'empty registry clears collection');
  writeFileSync(registryPath, JSON.stringify(registry, null, 2));
  const restoreDeadline = Date.now() + 10000;
  let restored = false;
  do {
    const response = await fetch(`${base}/gallery/ipc/`);
    restored = response.status === 200 && /<h1\b[^>]*>IPC<\/h1>/.test(await response.text());
    if (!restored) await new Promise(resolve => setTimeout(resolve, 200));
  } while (!restored && Date.now() < restoreDeadline);
  assert(restored, 'restore registry after empty collection');
  console.log('PASS: empty registry and subsequent restoration; original workspace registry unchanged.');
} finally {
  await server?.stop();
  rmSync(fixture, { recursive: true, force: true });
}
