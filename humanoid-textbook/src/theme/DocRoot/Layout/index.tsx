import React from 'react';
import DocRootLayout from '@theme-original/DocRoot/Layout';
import type DocRootLayoutType from '@theme/DocRoot/Layout';
import type { WrapperProps } from '@docusaurus/types';
import ChatWidget from '@site/src/components/ChatWidget';

type Props = WrapperProps<typeof DocRootLayoutType>;

export default function DocRootLayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <DocRootLayout {...props} />
      <ChatWidget />
    </>
  );
}
