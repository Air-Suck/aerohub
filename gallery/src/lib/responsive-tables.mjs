// Preserve native table semantics inside a keyboard-accessible scroll region.
export default {
  name: 'responsive-tables',
  element: {
    filter: ['table'],
    visit(node, context) {
      context.wrapNode(node, {
        type: 'element',
        tagName: 'div',
        properties: {
          className: ['table-scroll'],
          tabIndex: 0,
          role: 'region',
          ariaLabel: '表格，可横向滚动',
        },
        children: [],
      });
    },
  },
};
