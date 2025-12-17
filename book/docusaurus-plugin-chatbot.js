const path = require('path');

module.exports = function (context) {
  const { siteConfig } = context;

  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './src/client/chatbot.js')];
    },
  };
};