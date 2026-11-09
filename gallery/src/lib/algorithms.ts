export const integrationLabels = {
  not_started: '未迁移',
  in_progress: '迁移中',
  integrated: '已接入',
};

export const validationLabels = {
  unverified: '未登记验证证据',
  build: '构建验证',
  interface: '接口验证',
  simulation: '仿真验证',
  hardware: '真机验证',
};

export function sitePath(path = '') {
  return `${import.meta.env.BASE_URL.replace(/\/$/, '')}/${path.replace(/^\//, '')}`;
}
