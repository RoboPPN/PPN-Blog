## 分支策略

在实际的开发过程中，应该按照几个基本原则进行分支管理：

首先，`master`分支应该要保持非常稳定的版本，是给用户使用的版本，平时不能在上面写代码。

那在哪里写代码呢？

应该在`dev`分支上写代码，也就是说，`dev`分支是不稳定的，我们都在`dev`分支上干活，而且我们每个人都有属于自己的分支，例如我就有一个`mav-dev`的分支，写好代码后时不时往`dev`分支上合并就可以了。

那何时发布新版本给用户使用呢？

如果有专门的测试人员，在`dev`与`master`分支之间还有一个`test-dev`，等到`dev`目录已经有了待发布1.0版本的代码，测试人员就将`dev`合并到自己的分支上，经过测试，没有问题，再把`dev`分支合并到`master`上，在`master`分支上发布1.0版本的代码。

所以，团队的分支看起来就像这样：
![img error](img/git-branch.png)

## 创建与合并分支

- **查看分支**：`git branch`
- **创建分支**：`git branch <name>`
- **切换分支**：`git checkout <name>` 或者 `git switch <name>`
- **创建+切换分支**：`git checkout -b <name>` 或者 `git switch -c <name>`，该命令只是创建了本地的分支，但并没有创建远程仓库中的同名分支。要创建远程仓库中的分支，还需执行`git push -u origin <name>`
- **合并某分支到当前分支**：`git merge <name>` && `git push`
- 合并分支时，加上`--no-ff`参数就可以用普通模式合并，合并后的历史有分支，能看出来曾经做过合并，而fast forward合并就看不出来曾经做过合并。
- **删除分支**：`git branch -d <name>`,该命令只是删除了本地的分支，但并没有删除远程仓库中的同名分支。要删除远程仓库中的分支，还需执行`git push --delete dev`，最后，执行`git fetch --prune`来同步远程分支信息到本地，确保不再显示已删除的远程分支。
- **强行删除分支**：`git branch -D <new-branch-name>`
  对于创建的新分支，如果新分支还未合并到主分支，但领导说停止该分支的开发，如果使用`git branch -d`来删除的话，会出现：

  ```bash
  git branch -d new-branch-name
  error: The branch 'new-branch-name' is not fully merged.
  If you are sure you want to delete it, run 'git branch -D new-branch-name'.
  ```

  这里提示销毁失败，git提示`new-branch-name`分支还没有被合并，如果删除，将丢失掉修改，如果要强行删除，需要使用大写的`-D`参数。
