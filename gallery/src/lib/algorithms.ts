export const integrationLabels = {
  not_started: '未迁移',
  in_progress: '迁移中',
  integrated: '已接入',
};

export function sitePath(path = '') {
  return `${import.meta.env.BASE_URL.replace(/\/$/, '')}/${path.replace(/^\//, '')}`;
}
